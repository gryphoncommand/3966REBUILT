// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Set;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonUtils;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.FuelSim;
import frc.GryphonLib.AllianceFlipUtil;
import frc.GryphonLib.ShooterState;
import frc.robot.Constants.AlignmentConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.AI.StationaryBotInSimulation;
import frc.robot.commands.AlignToGoal;
import frc.robot.commands.AlignToGoalAuto;
import frc.robot.commands.AlignToTrench;
import frc.robot.commands.AllSystemsTest;
import frc.robot.commands.FlywheelSysID;
import frc.robot.commands.PassCommand;
import frc.robot.commands.PrepareSOTM;
import frc.robot.commands.RollerFloorSysID;
import frc.robot.commands.SetShooterToDefinedState;
import frc.robot.commands.SetToDashboardSpeeds;
import frc.robot.commands.Shoot;
import frc.robot.commands.ShootAllInHopper;
import frc.robot.commands.Intake.IntakeDeploy;
import frc.robot.commands.Intake.IntakeStow;
import frc.robot.commands.Intake.RunIntakeRollers;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.Drive.DriveIO;
import frc.robot.subsystems.Drive.DriveSubsystem;
import frc.robot.subsystems.Drive.SimDriveSubsystem;
import frc.robot.subsystems.Flywheel.FlywheelIO;
import frc.robot.subsystems.Flywheel.FlywheelSimTalonFX;
import frc.robot.subsystems.Flywheel.FlywheelTalonFX;
import frc.robot.subsystems.Indexer.Kicker;
import frc.robot.subsystems.Indexer.PreIndexer;
import frc.robot.subsystems.Intake.IntakeDeployIO;
import frc.robot.subsystems.Intake.IntakeDeploySimTalonFX;
import frc.robot.subsystems.Intake.IntakeDeploySparkFlex;
import frc.robot.subsystems.Intake.IntakeRollersTalonFX;

public class RobotContainer {

  // Subsystems
  private final DriveIO m_drive = Robot.isReal() ? new DriveSubsystem() : new SimDriveSubsystem();
  private final IntakeDeployIO m_intakeDeploy = Robot.isReal() ? new IntakeDeploySparkFlex() : new IntakeDeploySimTalonFX(m_drive);
  private final FlywheelIO m_flywheel = Robot.isReal() ? new FlywheelTalonFX() : new FlywheelSimTalonFX();
  private final Kicker m_kicker = new Kicker();
  private final PreIndexer m_preIndexer = new PreIndexer();
  private final IntakeRollersTalonFX m_intakeRollers = new IntakeRollersTalonFX();

  private Command runIntakeRollers = new RunIntakeRollers(m_intakeRollers);

  // Controllers
  private final CommandXboxController m_driverController =
      new CommandXboxController(OIConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController =
      new CommandXboxController(OIConstants.kOperatorControllerPort);

  private final SendableChooser<Command> autoChooser;
  private final SendableChooser<ShooterState> emergencyShotChooser = new SendableChooser<>();

  public RobotContainer() {
    new Blinkin();
    configureLogger();
    configureDefaultCommands();
    configureButtonBindings();
    configureStateTriggers();
    configureNamedCommands();
    if (Robot.isSimulation()){
      configureFuelSim();
      configureAIOpponents();
    }
    autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser.addOption("Flywheel SysID", new FlywheelSysID(m_flywheel).doAllSysID());
    autoChooser.addOption("Roller Floor SysID", new RollerFloorSysID(m_preIndexer).doAllSysID());
    autoChooser.addOption("Systems Test", new AllSystemsTest(m_drive, m_intakeDeploy, m_intakeRollers, m_kicker, m_preIndexer, m_flywheel).getSystemsTest());
    

    emergencyShotChooser.addOption("Default", ShooterConstants.kDefaultShooterState);
    emergencyShotChooser.addOption("Trench", ShooterConstants.kTrenchShotState);
    emergencyShotChooser.setDefaultOption("Tower", ShooterConstants.kTowerShotState);
    emergencyShotChooser.addOption("Corner", ShooterConstants.kCornerShotState);
    emergencyShotChooser.addOption("254 Shot", ShooterConstants.k254Shot);
    emergencyShotChooser.addOption("Juggle", ShooterConstants.kJuggleShooterState);

    SmartDashboard.putData("Auto Chooser", autoChooser);
    SmartDashboard.putData("Shot Chooser", emergencyShotChooser);
    SmartDashboard.putData("Shoot All", 
      new ParallelCommandGroup(
        new SetShooterToDefinedState(m_flywheel, ShooterConstants.kDefaultShooterState),
        new ShootAllInHopper(m_drive, m_flywheel, m_kicker, m_preIndexer, m_intakeDeploy, false, false)).withTimeout(5.5)
      );
    SmartDashboard.putData("Align To Goal", new AlignToGoal(m_drive, m_driverController, Constants.AlignmentConstants.BlueHubPose, false));
    SmartDashboard.putData("Return to Start", m_drive.goToPose(new Pose2d(new Translation2d(3.747, 7.419), new Rotation2d(Units.degreesToRadians(179.167)))));
  }

  private void configureDefaultCommands() {
    m_drive.setDefaultCommand(  
        new RunCommand(
            () -> {
              double forward = m_driverController.getLeftY();
              double strafe = m_driverController.getLeftX();
              double turn = m_driverController.getRightX();

              m_drive.drive(
                -MathUtil.applyDeadband(forward, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(strafe, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(turn, OIConstants.kDriveDeadband), DriveConstants.fieldOriented);
            },
            m_drive)
            .withName("Basic Drive"));
    m_preIndexer.setDefaultCommand(
      new RunCommand(()->{
        if (m_driverController.rightTrigger().getAsBoolean() || (m_driverController.povUp().getAsBoolean()) || m_operatorController.leftTrigger().getAsBoolean()){
          m_preIndexer.setVelocity(IndexerConstants.kPreIndexerSpeed);
        } else {
          m_preIndexer.setVelocity(0);
        }
      }, m_preIndexer)
    );

    m_flywheel.setDefaultCommand(
      new RunCommand(()->{
        m_flywheel.setVelocity(ShooterConstants.kDefaultFlywheelSpeed);
        if (ShooterConstants.kDefaultFlywheelSpeed == 0){
          m_flywheel.stop();
        }
      }, m_flywheel)
    );


    m_intakeRollers.setDefaultCommand(
      new RunCommand(()->{
        if (m_driverController.leftTrigger().getAsBoolean() || m_driverController.rightTrigger().getAsBoolean() || m_driverController.a().getAsBoolean()){
          m_intakeRollers.setVelocity(IntakeConstants.kIntakeSpeedRPM);
        } else {
          m_intakeRollers.setVelocity(0);
          m_intakeRollers.set(0);
        }
      }, m_intakeRollers)
    );
  }

  private void configureButtonBindings() {
    // Driver bindings
    m_driverController.start().onTrue(
      new InstantCommand(()->{
        m_drive.zeroHeading();
        AlignmentConstants.HubPose = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red ? AlignmentConstants.RedHubPose : AlignmentConstants.BlueHubPose;
        AlignmentConstants.PassingPoseOutpost = (new Pose2d(AllianceFlipUtil.applyX(0.812), 2.2688, new Rotation2d()));
        AlignmentConstants.PassingPoseDepot = (new Pose2d(AllianceFlipUtil.applyX(0.812), 5.707, new Rotation2d()));
      }, m_drive));
    

    m_driverController.rightBumper()
      .whileTrue(new RepeatCommand(new DeferredCommand(()->
        new ParallelCommandGroup(
            new AlignToGoal(m_drive, m_driverController, AlignmentConstants.HubPose, true),
            new PrepareSOTM(m_flywheel, m_drive, AlignmentConstants.HubPose, ShooterConstants.RealShootingValuesLow)
        ), Set.of(m_drive, m_flywheel))))
      .onFalse(new RunCommand(()->m_flywheel.stop(), m_flywheel));

    m_driverController.rightTrigger()
        .whileTrue(runIntakeRollers)
        .whileTrue(new Shoot(m_drive, m_flywheel, m_kicker, m_preIndexer, m_intakeDeploy, false, true, m_driverController.leftTrigger().negate()::getAsBoolean).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    m_driverController.leftTrigger()
      .whileTrue(
        Commands.either(
            new IntakeDeploy(m_intakeDeploy),
            new InstantCommand(),
            () -> m_intakeDeploy.getPosition() > 20
        )
      )
      .whileTrue(runIntakeRollers);

    m_driverController.leftBumper().whileTrue(new IntakeStow(m_intakeDeploy));
    m_driverController.x()
    .whileTrue(
      new RepeatCommand(new DeferredCommand(()->
        new PassCommand(m_drive, m_driverController, m_flywheel)
        , Set.of(m_drive, m_flywheel)
      ))
    )
    .onFalse(new RunCommand(()->m_flywheel.stop(), m_flywheel));

    m_driverController.b().whileTrue(new AlignToTrench(m_drive, m_driverController));
    m_driverController.a()
      .whileTrue(
        new DeferredCommand(()->new SetShooterToDefinedState(m_flywheel,
          emergencyShotChooser.getSelected()
        ), Set.of(m_flywheel))
      )
      .whileTrue(new Shoot(m_drive, m_flywheel, m_kicker, m_preIndexer, m_intakeDeploy, false, false, m_driverController.leftTrigger().negate()::getAsBoolean).withInterruptBehavior(InterruptionBehavior.kCancelIncoming))
      .onFalse(new SetShooterToDefinedState(m_flywheel, ShooterConstants.kShooterStowState).withTimeout(0.1));
    m_driverController.y()
      .whileTrue(new RunCommand(()->m_intakeRollers.set(-0.8), m_intakeRollers))
      .whileTrue(new RunCommand(()->m_preIndexer.set(-0.8), m_preIndexer))
      .onFalse(new RunCommand(()->m_intakeRollers.set(0.0), m_intakeRollers))
      .onFalse(new RunCommand(()->m_preIndexer.set(0.0), m_preIndexer));
    m_driverController.povRight().whileTrue(new RunCommand(()->{
      m_drive.setX();
    }, m_drive));

    m_driverController.povLeft().onTrue(
      new DeferredCommand(
        ()->new InstantCommand(()->Logger.recordOutput("Current Shooter State", "new ShooterState(" +  String.valueOf(PhotonUtils.getDistanceToPose(m_drive.getCurrentPose().plus(ShooterConstants.kRobotToShooter), AlignmentConstants.HubPose)) + ", " + String.valueOf(ShooterConstants.kFixedHoodAngleDeg) + ", " + String.valueOf(m_flywheel.getVelocity()) + ", measuredShotTime)")),
        Set.of()
    ));
    
    m_operatorController.rightTrigger()
        .whileTrue(new SetShooterToDefinedState(m_flywheel, ShooterConstants.kDefaultShooterState))
        .whileTrue(new Shoot(m_drive, m_flywheel, m_kicker, m_preIndexer, m_intakeDeploy, false, false, ()->true).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    m_operatorController.x().toggleOnTrue(new SetToDashboardSpeeds(m_flywheel));
    

    m_operatorController.leftTrigger()
      .whileTrue(runIntakeRollers)
      .whileTrue(new RunCommand(()->m_preIndexer.setVelocity(IndexerConstants.kPreIndexerSpeed), m_preIndexer))
      .onFalse(new RunCommand(()->m_preIndexer.setVelocity(0), m_preIndexer));
    m_operatorController.leftBumper()
      .whileTrue(new RunCommand(()->m_intakeDeploy.set(0.2), m_intakeDeploy))
      .onFalse(new RunCommand(()->m_intakeDeploy.set(0.0), m_intakeDeploy));
    m_operatorController.rightBumper()
      .whileTrue(new RunCommand(()->m_intakeDeploy.set(-0.2), m_intakeDeploy))
      .onFalse(new RunCommand(()->m_intakeDeploy.set(0.0), m_intakeDeploy));

    m_operatorController.a()
    .whileTrue(new RunCommand(()->m_kicker.set(-0.2), m_kicker)).onFalse(new RunCommand(()->m_kicker.set(0.0), m_kicker))
    .whileTrue(new RunCommand(()->m_preIndexer.set(0.05), m_preIndexer)).onFalse(new RunCommand(()->m_preIndexer.set(0.0), m_preIndexer));
    

    m_operatorController.y()
      .whileTrue(new RunCommand(()->m_intakeRollers.set(-0.8), m_intakeRollers))
      .whileTrue(new RunCommand(()->m_preIndexer.set(-0.8), m_preIndexer))
      .onFalse(new RunCommand(()->m_intakeRollers.set(0.0), m_intakeRollers))
      .onFalse(new RunCommand(()->m_preIndexer.set(0.0), m_preIndexer));
    

    m_operatorController.start().onTrue(
      new InstantCommand(()->{
        Pose2d shooterPose = m_drive.getCurrentPose().plus(ShooterConstants.kRobotToShooter);
        SmartDashboard.putString("Current Shooter State", "new ShooterState(" +  String.valueOf(PhotonUtils.getDistanceToPose(shooterPose, AlignmentConstants.HubPose)) + ", " + String.valueOf(ShooterConstants.kFixedHoodAngleDeg) + ", " + String.valueOf(m_flywheel.getVelocity()) + ", measuredShotTime)");
      })
    );

  }
 

  private void configureStateTriggers() {
    // Trigger inAllianceZone = new Trigger(
    //   ()->{
    //     double x = m_drive.getCurrentPose().getX();
    //     return DriverStation.getAlliance().get() == Alliance.Red ? x < AlignmentConstants.RedAllianceZoneEnd.getX() : x > AlignmentConstants.BlueAllianceZoneEnd.getX();
    //   }
    // );


    // Trigger ShooterReady = new Trigger(()->(m_flywheel.atTarget(50)));
    // Trigger Aligned = new Trigger(m_drive::getAligned);

    // Debouncer ShootDebouncer = new Debouncer(0.5);
    // Trigger ReadyToShoot = new Trigger(()->ShootDebouncer.calculate(Aligned.getAsBoolean() && ShooterReady.getAsBoolean()));

    // ReadyToShoot.onChange(new InstantCommand(()->SmartDashboard.putBoolean("Ready To Shoot", ReadyToShoot.getAsBoolean())));
    
  }

  private void configureFuelSim(){
    FuelSim instance = FuelSim.getInstance();
    try {
      instance.registerRobot(
        0.635, // from left to right
        0.737, // from front to back
        Units.inchesToMeters(6), // from floor to top of bumpers
        ((SimDriveSubsystem)m_drive)::getRealPoseSim, // Supplier<Pose2d> of robot pose
        m_drive::getCurrentSpeedsFieldRelative // Supplier<ChassisSpeeds> of field-centric chassis speeds
      );
    } catch (Exception e){
      instance.registerRobot(
        0.635, // from left to right
        0.737, // from front to back
        Units.inchesToMeters(6), // from floor to top of bumpers
        m_drive::getCurrentPose, // Supplier<Pose2d> of robot pose
        m_drive::getCurrentSpeedsFieldRelative // Supplier<ChassisSpeeds> of field-centric chassis speeds
      );
    }
    
    

    instance.registerIntake(
        -(0.635 + 2*(IntakeConstants.kIntakeLengthMeters*0.9))/2, -(0.635)/2, -(0.6)/2, (0.65)/2, // robot-centric coordinates for bounding box
        () -> (SmartDashboard.getBoolean("Intake Deployed", true) && !m_preIndexer.isFull()), // (optional) BooleanSupplier for whether the intake should be active at a given moment
        new Runnable() {public void run() {m_preIndexer.addBall();};}); // (optional) Runnable called whenever a fuel is intaked

    instance.start();

    // Performance tuning for sim
    instance.setLogEveryNTicks(3); // 25 Hz fuel pose logging
    instance.disableProfiling();

    SmartDashboard.putData("Reset Fuel", Commands.runOnce(() -> {
          FuelSim.getInstance().clearFuel();
          FuelSim.getInstance().spawnStartingFuel();
          FuelSim.getInstance().reserveFuelForRobot((int) m_preIndexer.getBalls());
      })
      .withName("Reset Fuel")
      .ignoringDisable(true));
    
      CommandScheduler.getInstance().schedule(Commands.runOnce(() -> {
          FuelSim.getInstance().clearFuel();
          FuelSim.getInstance().spawnStartingFuel();
      })
      .withName("Reset Fuel")
      .ignoringDisable(true));

    instance.enableAirResistance();
    CommandScheduler.getInstance().schedule(new AlignToGoal(m_drive, m_driverController, AlignmentConstants.HubPose, true).withTimeout(0.1));
  }

  private void configureNamedCommands(){
    NamedCommands.registerCommand("Shoot All Balls", new ShootAllInHopper(m_drive, m_flywheel, m_kicker, m_preIndexer, m_intakeDeploy).withTimeout(4.0).raceWith(new RunIntakeRollers(m_intakeRollers)));
    NamedCommands.registerCommand("Speedup Flywheel", new DeferredCommand(()->new PrepareSOTM(m_flywheel, m_drive, AlignmentConstants.HubPose, ShooterConstants.RealShootingValuesLow), Set.of(m_flywheel)));
    NamedCommands.registerCommand("Prepare to Shoot", 
    new DeferredCommand(()->      new ParallelCommandGroup(
        new PrepareSOTM(m_flywheel, m_drive, AlignmentConstants.HubPose, ShooterConstants.RealShootingValuesLow),
        new AlignToGoalAuto(m_drive, AlignmentConstants.HubPose, true)
      ),
      Set.of(m_flywheel, m_drive))
    );
    NamedCommands.registerCommand("Align to Shoot", new AlignToGoalAuto(m_drive, AlignmentConstants.HubPose, true));
    NamedCommands.registerCommand("Outpost Intake", new InstantCommand(()->{for (int i = 0; i < 24; i++){m_preIndexer.addBall();}}));
    NamedCommands.registerCommand("Outpost Spawn", new InstantCommand(()->FuelSim.getInstance().spawnOutpostFuel()));
    NamedCommands.registerCommand("Deploy Intake", new IntakeDeploy(m_intakeDeploy));
    NamedCommands.registerCommand("Stow Intake", new IntakeStow(m_intakeDeploy));
    NamedCommands.registerCommand("Run Intake", new RunIntakeRollers(m_intakeRollers));
    NamedCommands.registerCommand("Stop", new RunCommand(()->m_drive.stop(), m_drive).withTimeout(1));
  }

  private void configureLogger(){
    Logger.recordOutput(
      "ZeroedComponentPoses", 
      new Pose3d[] {
        new Pose3d(),
        new Pose3d(),
        new Pose3d()
      }
    );
  }

  public void configureAIOpponents(){
    try {      
      Logger.recordOutput("Drive/AI Status", "Started Creating AI 0");
      new StationaryBotInSimulation(3);
      // new HybridBotInSimulation(3, ((SimDriveSubsystem)m_drive)::getRealPoseSim, Alliance.Red);
  } catch (Exception e){
      Logger.recordOutput("Drive/AI Status", "Failed Creating AI 0, " + e.getMessage());
    }
  }
    
  /** Returns the autonomous command. */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public int getSimBallCount() {
    return (int) m_preIndexer.getBalls();
  }
}
