// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.FuelSim;
import frc.GryphonLib.ShooterState;
import frc.littletonUtils.AllianceFlipUtil;
import frc.robot.Constants.AlignmentConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.AlignToGoal;
import frc.robot.commands.AlignToTrench;
import frc.robot.commands.AllSystemsTest;
import frc.robot.commands.AutoClimbCommand;
import frc.robot.commands.DeployClimber;
import frc.robot.commands.FlywheelSysID;
import frc.robot.commands.HomeHood;
import frc.robot.commands.PassCommand;
import frc.robot.commands.PrepareSOTM;
import frc.robot.commands.SetShooterToDefinedState;
import frc.robot.commands.SetToDashboardSpeeds;
import frc.robot.commands.Shoot;
import frc.robot.commands.ShootAllInHopper;
import frc.robot.commands.Indexing.FeedShooterFactory;
import frc.robot.commands.Indexing.RunPreIndexer;
import frc.robot.commands.Intake.IntakeDeploy;
import frc.robot.commands.Intake.IntakeStow;
import frc.robot.commands.Intake.RunIntakeRollers;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Climber.ClimberIO;
import frc.robot.subsystems.Climber.ClimberSimTalonFX;
import frc.robot.subsystems.Climber.ClimberTalonFX;
import frc.robot.subsystems.Flywheel.FlywheelIO;
import frc.robot.subsystems.Flywheel.FlywheelSimTalonFX;
import frc.robot.subsystems.Flywheel.FlywheelSparkFlex;

import java.util.Set;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonUtils;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Hood.*;
import frc.robot.subsystems.Indexer.Kicker;
import frc.robot.subsystems.Indexer.PreIndexer;
import frc.robot.subsystems.Indexer.Spindexer;
import frc.robot.subsystems.Intake.IntakeDeployIO;
import frc.robot.subsystems.Intake.IntakeDeploySimTalonFX;
import frc.robot.subsystems.Intake.IntakeDeploySparkFlex;
import frc.robot.subsystems.Intake.IntakeRollersTalonFX;

public class RobotContainer {

  // Subsystems
  private final DriveSubsystem m_drive = new DriveSubsystem();
  private final IntakeDeployIO m_intakeDeploy = Robot.isReal() ? new IntakeDeploySparkFlex() : new IntakeDeploySimTalonFX();
  private final FlywheelIO m_flywheel = Robot.isReal() ? new FlywheelSparkFlex() : new FlywheelSimTalonFX();
  private final Kicker m_kicker = new Kicker();
  private final PreIndexer m_preIndexer = new PreIndexer();
  private final Spindexer m_spindexer = new Spindexer();
  private final HoodIO m_hood = Robot.isReal() ? new HoodTalonFX() : new HoodSimTalonFX();
  private final IntakeRollersTalonFX m_intakeRollers = new IntakeRollersTalonFX();
  private final ClimberIO m_climber = Robot.isReal() ? new ClimberTalonFX() : new ClimberSimTalonFX();

  private Command runIntakeRollers = new RunIntakeRollers(m_intakeRollers);

  // Controllers
  private final CommandXboxController m_driverController =
      new CommandXboxController(OIConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController =
      new CommandXboxController(OIConstants.kOperatorControllerPort);

  private final SendableChooser<Command> autoChooser;
  private final SendableChooser<ShooterState> emergencyShotChooser = new SendableChooser<>();

  public RobotContainer() {
    new Vision();
    configureLogger();
    configureDefaultCommands();
    configureButtonBindings();
    configureStateTriggers();
    configureNamedCommands();
    if (Robot.isSimulation()){
      configureFuelSim();
    }
    autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser.addOption("Flywheel SysID", new FlywheelSysID(m_flywheel).doAllSysID());
    autoChooser.addOption("Systems Test", new AllSystemsTest(m_drive, m_intakeDeploy, m_intakeRollers, m_kicker, m_preIndexer, m_spindexer, m_flywheel, m_climber, m_hood).getSystemsTest());
    

    emergencyShotChooser.setDefaultOption("Default", ShooterConstants.kDefaultShooterState);
    emergencyShotChooser.addOption("Tower", ShooterConstants.kTowerShotState);
    emergencyShotChooser.addOption("Trench", ShooterConstants.kTrenchShotState);
    emergencyShotChooser.addOption("Corner", ShooterConstants.kCornerShotState);
    SmartDashboard.putData("Auto Chooser", autoChooser);
    SmartDashboard.putData("Shot Chooser", emergencyShotChooser);
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
                -MathUtil.applyDeadband(turn, OIConstants.kDriveDeadband), true);
            },
            m_drive)
            .withName("Basic Drive"));
    m_preIndexer.setDefaultCommand(
      new RunCommand(()->{
        if (m_driverController.leftTrigger().getAsBoolean()){
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
  }

  private void configureButtonBindings() {
    FeedShooterFactory testFactory = new FeedShooterFactory(m_kicker, m_preIndexer, m_spindexer);
    // Driver bindings
    m_driverController.start().onTrue(
      new InstantCommand(()->{
        m_drive.zeroHeading();
        AlignmentConstants.HubPose = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red ? AlignmentConstants.RedHubPose : AlignmentConstants.BlueHubPose;
        AlignmentConstants.PassingPoseOutpost = AllianceFlipUtil.apply(new Pose2d(2.412, 2.2688, new Rotation2d()));
        AlignmentConstants.PassingPoseDepot = AllianceFlipUtil.apply(new Pose2d(2.412, 5.707, new Rotation2d()));
      }, m_drive));
    // m_driverController.povRight().onTrue(
    //   new InstantCommand(()->{
    //     Pose2d shooterPose = m_drive.getCurrentPose().plus(ShooterConstants.kRobotToShooter);
    //     SmartDashboard.putString("Current Shooter State", "new ShooterState(" +  String.valueOf(PhotonUtils.getDistanceToPose(shooterPose, AlignmentConstants.HubPose)) + ", " + String.valueOf(m_hood.getAngle()) + ", " + String.valueOf(m_flywheel.getVelocity()) + ", measuredShotTime)");
    //   })
    // );

    m_driverController.rightBumper()
      .whileTrue(new RepeatCommand(new DeferredCommand(()->
        new ParallelCommandGroup(
            new AlignToGoal(m_drive, m_driverController, AlignmentConstants.HubPose, true),
            new PrepareSOTM(m_hood, m_flywheel, m_drive, AlignmentConstants.HubPose, ShooterConstants.RealShootingValuesHigh)
        ), Set.of(m_drive, m_hood, m_flywheel))))
      .onFalse(new RunCommand(()->m_flywheel.stop(), m_flywheel))
      .onFalse(new HomeHood(m_hood));

    m_driverController.rightTrigger()
        .whileTrue(runIntakeRollers)
        // .whileTrue(new AgitateIntake(m_intakeDeploy))
        .whileTrue(new Shoot(m_drive, m_hood, m_flywheel, m_intakeRollers, m_kicker, m_preIndexer, m_spindexer, m_intakeDeploy, false, true).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    m_driverController.leftTrigger()
      .whileTrue(
        Commands.either(
            new IntakeDeploy(m_intakeDeploy),
            new InstantCommand(),
            () -> m_intakeDeploy.getPosition() > 20
        )
      )
      .whileTrue(runIntakeRollers)
      .whileTrue(new RunPreIndexer(m_preIndexer));
      
    m_driverController.leftBumper().whileTrue(new IntakeStow(m_intakeDeploy));
    m_driverController.x()
    .whileTrue(
      new RepeatCommand(new DeferredCommand(()->
        new PassCommand(m_drive, m_driverController, m_hood, m_flywheel)
        , Set.of(m_drive, m_hood, m_flywheel)
      ))
    )
    .onFalse(new RunCommand(()->m_flywheel.stop(), m_flywheel))
    .onFalse(new HomeHood(m_hood));

    m_driverController.b().whileTrue(new AlignToTrench(m_drive, m_driverController));
    m_driverController.a()
      .whileTrue(
        new DeferredCommand(()->new SetShooterToDefinedState(m_hood, m_flywheel,
          emergencyShotChooser.getSelected()
        ), Set.of(m_hood, m_flywheel))
      )
      .whileTrue(new Shoot(m_drive, m_hood, m_flywheel, m_intakeRollers, m_kicker, m_preIndexer, m_spindexer, m_intakeDeploy, false, false).withInterruptBehavior(InterruptionBehavior.kCancelIncoming))
      .onFalse(new SetShooterToDefinedState(m_hood, m_flywheel, ShooterConstants.kShooterStowState).withTimeout(0.1));
    m_driverController.povLeft().whileTrue(new HomeHood(m_hood).alongWith(new RunCommand(()->m_flywheel.set(0), m_flywheel)));
    m_driverController.y()
      .whileTrue(new RunCommand(()->m_intakeRollers.set(-0.3), m_intakeRollers))
      .onFalse(new RunCommand(()->m_intakeRollers.set(0.0), m_intakeRollers));
    m_driverController.povDown()
    .toggleOnTrue(new DeployClimber(m_climber))
    .onTrue(new RunCommand(()->m_flywheel.set(0), m_flywheel));
    m_driverController.povUp().whileTrue(new RunCommand(()->m_climber.set(-0.5), m_climber)).onFalse(new RunCommand(()->m_climber.set(0), m_climber));
    m_driverController.povRight().toggleOnTrue(new AutoClimbCommand(m_drive, m_climber));
    
    
    m_operatorController.rightTrigger()
        .whileTrue(new SetShooterToDefinedState(m_hood, m_flywheel, ShooterConstants.kDefaultShooterState))
        .whileTrue(new Shoot(m_drive, m_hood, m_flywheel, m_intakeRollers, m_kicker, m_preIndexer, m_spindexer, m_intakeDeploy, false, false).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    m_operatorController.x().toggleOnTrue(new SetToDashboardSpeeds(m_hood, m_flywheel));
    m_operatorController.povUp().whileTrue(new RunCommand(()->m_climber.set(0.5), m_climber)).onFalse(new RunCommand(()->m_climber.set(0.0), m_climber));
    m_operatorController.povDown().whileTrue(new RunCommand(()->m_climber.set(-0.5), m_climber)).onFalse(new RunCommand(()->m_climber.set(0.0), m_climber));
    m_operatorController.povLeft().onTrue(new RunCommand(()->m_climber.setEncoderPosition(0), m_climber));
    

    m_operatorController.leftTrigger()
      .whileTrue(runIntakeRollers)
      .whileTrue(new RunCommand(()->m_preIndexer.setVelocity(IndexerConstants.kPreIndexerSpeed), m_preIndexer))
      .onFalse(new RunCommand(()->m_preIndexer.setVelocity(0), m_preIndexer));
    m_operatorController.leftBumper()
      .whileTrue(new RunCommand(()->testFactory.start(true), m_kicker, m_preIndexer, m_spindexer))
      .onFalse(new RunCommand(()->testFactory.stop(), m_kicker, m_preIndexer, m_spindexer))
      .whileTrue(runIntakeRollers)
      // .whileTrue(new AgitateIntake(m_intakeDeploy))
      .whileTrue(new RepeatCommand(new InstantCommand(()->testFactory.periodic())));

    m_operatorController.start().onTrue(
      new InstantCommand(()->{
        Pose2d shooterPose = m_drive.getCurrentPose().plus(ShooterConstants.kRobotToShooter);
        SmartDashboard.putString("Current Shooter State", "new ShooterState(" +  String.valueOf(PhotonUtils.getDistanceToPose(shooterPose, AlignmentConstants.HubPose)) + ", " + String.valueOf(m_hood.getAngle()) + ", " + String.valueOf(m_flywheel.getVelocity()) + ", measuredShotTime)");
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


    // Trigger ShooterReady = new Trigger(()->(m_flywheel.atTarget(50) && m_hood.atTarget(5)));
    // Trigger Aligned = new Trigger(m_drive::getAligned);

    // Debouncer ShootDebouncer = new Debouncer(0.5);
    // Trigger ReadyToShoot = new Trigger(()->ShootDebouncer.calculate(Aligned.getAsBoolean() && ShooterReady.getAsBoolean()));

    // ReadyToShoot.onChange(new InstantCommand(()->SmartDashboard.putBoolean("Ready To Shoot", ReadyToShoot.getAsBoolean())));
    
  }

  private void configureFuelSim(){
    FuelSim instance = FuelSim.getInstance();
    instance.registerRobot(
      0.635, // from left to right
      0.737, // from front to back
      Units.inchesToMeters(6), // from floor to top of bumpers
      m_drive::getCurrentPose, // Supplier<Pose2d> of robot pose
      m_drive::getCurrentSpeedsFieldRelative // Supplier<ChassisSpeeds> of field-centric chassis speeds
    );

    instance.registerIntake(
        -(0.635 + 2*(IntakeConstants.kIntakeLengthMeters*0.9))/2, -(0.635)/2, -(0.7)/2, (0.7)/2, // robot-centric coordinates for bounding box
        () -> (SmartDashboard.getBoolean("Intake Deployed", true) && !m_spindexer.isFull()), // (optional) BooleanSupplier for whether the intake should be active at a given moment
        new Runnable() {public void run() {m_spindexer.addBall();};}); // (optional) Runnable called whenever a fuel is intaked

    instance.start();

    SmartDashboard.putData("Reset Fuel", Commands.runOnce(() -> {
          FuelSim.getInstance().clearFuel();
          FuelSim.getInstance().spawnStartingFuel();
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
  }

  private void configureNamedCommands(){
    NamedCommands.registerCommand("Shoot All Balls", new ShootAllInHopper(m_drive, m_hood, m_flywheel, m_intakeRollers, m_kicker, m_preIndexer, m_spindexer));
    NamedCommands.registerCommand("Prepare to Shoot", 
      new ParallelCommandGroup(
        new AlignToGoal(m_drive, m_driverController, AlignmentConstants.HubPose, true),
        new PrepareSOTM(m_hood, m_flywheel, m_drive, AlignmentConstants.HubPose, ShooterConstants.RealShootingValuesHigh)
      )
    );
    NamedCommands.registerCommand("Deploy Intake", new IntakeDeploy(m_intakeDeploy));
    NamedCommands.registerCommand("Stow Intake", new IntakeStow(m_intakeDeploy));
    NamedCommands.registerCommand("Run Intake", new RunCommand(()->m_intakeRollers.setVelocity(IntakeConstants.kIntakeSpeedRPM), m_intakeRollers).finallyDo(()->m_intakeRollers.set(0)));
    NamedCommands.registerCommand("Home Hood", new HomeHood(m_hood));
    NamedCommands.registerCommand("Climb", new DeployClimber(m_climber));
    NamedCommands.registerCommand("Deploy Climber", new RunCommand(()->m_climber.setPosition(ClimberConstants.kFullUpPosition), m_climber));
    NamedCommands.registerCommand("Stop", new RunCommand(()->m_drive.stop(), m_drive).withTimeout(1));
    NamedCommands.registerCommand("Auto Climb", new AutoClimbCommand(m_drive, m_climber));
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
    
  /** Returns the autonomous command. */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
