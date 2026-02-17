// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.FuelSim;
import frc.GryphonLib.ShooterState;
import frc.robot.Constants.AlignmentConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.AlignToGoal;
import frc.robot.commands.FlywheelSysID;
import frc.robot.commands.HomeHood;
import frc.robot.commands.PrepareSOTM;
import frc.robot.commands.ResetShooter;
import frc.robot.commands.SetShooterToDefinedState;
import frc.robot.commands.SetToDashboardSpeeds;
import frc.robot.commands.Shoot;
import frc.robot.commands.PassOrShootCommand;
import frc.robot.commands.ShootAllInHopper;
import frc.robot.commands.Intake.HomeIntake;
import frc.robot.commands.Intake.IntakeDeploy;
import frc.robot.commands.Intake.IntakeStow;
import frc.robot.commands.Intake.RunIntakeRollers;
import frc.robot.subsystems.DriveSubsystem;
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
import edu.wpi.first.math.util.Units;
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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Hood.*;
import frc.robot.subsystems.Intake.IntakeDeployIO;
import frc.robot.subsystems.Intake.IntakeDeploySimTalonFX;
import frc.robot.subsystems.Intake.IntakeDeploySparkFlex;
import frc.robot.subsystems.Intake.IntakeRollersTalonFX;

public class RobotContainer {

  // Subsystems
  private final DriveSubsystem m_drive = new DriveSubsystem();
  private final IntakeDeployIO m_intakeDeploy = Robot.isReal() ? new IntakeDeploySparkFlex() : new IntakeDeploySimTalonFX();
  private final FlywheelIO m_flywheel = Robot.isReal() ? new FlywheelSparkFlex() : new FlywheelSimTalonFX();
  private final HoodIO m_hood = Robot.isReal() ? new HoodTalonFX() : new HoodSimTalonFX();
  private final IntakeRollersTalonFX m_intakeRollers = new IntakeRollersTalonFX();

  private Command runIntakeRollers = new RunIntakeRollers(m_intakeRollers);

  // Controllers
  private final CommandXboxController m_driverController =
      new CommandXboxController(OIConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController =
      new CommandXboxController(OIConstants.kOperatorControllerPort);

  private final SendableChooser<Command> autoChooser;

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
    autoChooser.addOption("Flywheel SysID", new FlywheelSysID(m_flywheel).SysIDDynamic());
    SmartDashboard.putData("Auto Chooser", autoChooser);
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
  }

  private void configureButtonBindings() {
    // Driver bindings
    m_operatorController.start().onTrue(new InstantCommand(()->m_drive.zeroHeading(), m_drive));

    m_driverController.rightBumper()
      .whileTrue(new RepeatCommand(new DeferredCommand(()->new PassOrShootCommand(m_drive, m_driverController, m_hood, m_flywheel), Set.of(m_drive, m_hood, m_flywheel))))
      .onFalse(new RunCommand(()->m_flywheel.setVelocity(1000), m_flywheel))
      .onFalse(new HomeHood(m_hood));

    m_driverController.rightTrigger().whileTrue(new Shoot(m_drive, m_hood, m_flywheel, m_intakeRollers, 0));
    m_driverController.leftTrigger()
      .onTrue(new IntakeDeploy(m_intakeDeploy))
      .whileTrue(runIntakeRollers);
    m_driverController.leftBumper().whileTrue(new IntakeStow(m_intakeDeploy));
    m_driverController.a().whileTrue(new HomeIntake(m_intakeDeploy));
    m_driverController.b().whileTrue(new HomeHood(m_hood));
    

    m_operatorController.a().whileTrue(new HomeHood(m_hood));
    m_operatorController.x().toggleOnTrue(new SetToDashboardSpeeds(m_hood, m_flywheel));

    SequentialCommandGroup testHood = new SequentialCommandGroup(
      new SetShooterToDefinedState(m_hood, m_flywheel, new ShooterState(1, 45, 3000, 0.7)),
      new SetShooterToDefinedState(m_hood, m_flywheel, new ShooterState(1, 50, 4000, 0.7)),
      new SetShooterToDefinedState(m_hood, m_flywheel, new ShooterState(1, 30, 2000, 0.7)),
      new SetShooterToDefinedState(m_hood, m_flywheel, new ShooterState(1, 45, 3500, 0.7)),
      new ResetShooter(m_hood, m_flywheel)
    );

    m_operatorController.y().onTrue(testHood);

    m_operatorController.start().onTrue(
      new InstantCommand(()->{
        Pose2d shooterPose = m_drive.getCurrentPose().plus(ShooterConstants.kRobotToShooter);
        Logger.recordOutput("Tuning/Current Shooter State", "new ShooterState(" +  String.valueOf(PhotonUtils.getDistanceToPose(shooterPose, AlignmentConstants.HubPose)) + ", " + String.valueOf(m_hood.getAngle()) + ", " + String.valueOf(m_flywheel.getVelocity()) + ", measuredShotTime)");
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
        -(0.635 + 2*IntakeConstants.kIntakeLengthMeters)/2, -(0.635)/2, -(0.737)/2, (0.737)/2, // robot-centric coordinates for bounding box
        () -> (SmartDashboard.getBoolean("Intake Deployed", true) && !m_intakeRollers.isFull()), // (optional) BooleanSupplier for whether the intake should be active at a given moment
        new Runnable() {public void run() {m_intakeRollers.addBall();};}); // (optional) Runnable called whenever a fuel is intaked

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
  }

  private void configureNamedCommands(){
    NamedCommands.registerCommand("Shoot All Balls", new ShootAllInHopper(m_drive, m_hood, m_flywheel, m_intakeRollers));
    NamedCommands.registerCommand("Prepare to Shoot", 
      new ParallelCommandGroup(
        new AlignToGoal(m_drive, m_driverController, AlignmentConstants.HubPose, true),
        new PrepareSOTM(m_hood, m_flywheel, m_drive, ShooterConstants.RealShootingValues)
      )
    );
    NamedCommands.registerCommand("Deploy Intake", new IntakeDeploy(m_intakeDeploy));
    NamedCommands.registerCommand("Stow Intake", new IntakeStow(m_intakeDeploy));
    NamedCommands.registerCommand("Run Intake", runIntakeRollers);
    NamedCommands.registerCommand("Home Hood", new HomeHood(m_hood));
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
    
  /** Returns the autonomous command. */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
