// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AlignmentConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.AlignToGoal;
import frc.robot.commands.HomeHood;
import frc.robot.commands.PrepareSOTM;
import frc.robot.commands.PrepareToShoot;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Flywheel.FlywheelIO;
import frc.robot.subsystems.Flywheel.FlywheelSimTalonFX;
import frc.robot.subsystems.Flywheel.FlywheelSparkFlex;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Hood.*;
import frc.robot.subsystems.Intake.IntakeRollersSparkFlex;

public class RobotContainer {

  // Subsystems
  private final DriveSubsystem m_drive = new DriveSubsystem();
  // private final FlywheelIO m_flywheel = Robot.isReal() ? new FlywheelSparkFlex() : new FlywheelSimTalonFX();
  // private final HoodIO m_hood = Robot.isReal() ? new HoodTalonFX() : new HoodSimTalonFX();
  // private final IntakeRollersSparkFlex m_intakeRollers = new IntakeRollersSparkFlex();

  // Controllers
  private final CommandXboxController m_driverController =
      new CommandXboxController(OIConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController =
      new CommandXboxController(OIConstants.kOperatorControllerPort);

  public RobotContainer() {
    new Vision();
    configureDefaultCommands();
    configureButtonBindings();
    configureStateTriggers();
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
            m_drive));

    // m_intakeRollers.setDefaultCommand(
    //   new RunCommand(()->{
    //     m_intakeRollers.stop();
    //   }, m_intakeRollers)
    // );
  }

  private void configureButtonBindings() {
    // Driver bindings
    m_driverController.start().onTrue(new InstantCommand(()->m_drive.zeroHeading(), m_drive));
    m_driverController.rightBumper().whileTrue(new AlignToGoal(m_drive, m_driverController, DriverStation.getAlliance().get() == Alliance.Red ? AlignmentConstants.RedHubPose : AlignmentConstants.BlueHubPose, true));
    // m_driverController.leftBumper().whileTrue(new RepeatCommand(new PrepareSOTM(m_hood, m_flywheel, m_drive, ShooterConstants.FakeValues)));
   
    // m_driverController.b().whileTrue(new HomeHood(m_hood));
    // m_driverController.leftTrigger().whileTrue(new RunCommand(()->m_intakeRollers.intake(), m_intakeRollers));
    

    // m_operatorController.a().onTrue(new InstantCommand(m_drive::stop, m_drive));
    // m_operatorController.y().onTrue(new InstantCommand(m_drive::setX, m_drive));
  }


  private void configureStateTriggers() {
    // Trigger ShooterReady = new Trigger(()->(m_flywheel.atTarget(50) && m_hood.atTarget(5)));
    // Trigger Aligned = new Trigger(m_drive::getAligned);

    // Debouncer ShootDebouncer = new Debouncer(0.5);
    // Trigger ReadyToShoot = new Trigger(()->ShootDebouncer.calculate(Aligned.getAsBoolean() && ShooterReady.getAsBoolean()));

    // ReadyToShoot.onChange(new InstantCommand(()->SmartDashboard.putBoolean("Ready To Shoot", ReadyToShoot.getAsBoolean())));
    
  }
    

  /** Returns the autonomous command. */
  public Command getAutonomousCommand() {
    // return m_drive.PathToPose(new Pose2d(new Translation2d(4, 6), new Rotation2d(3*Math.PI/4)), 0).andThen(
    //   m_drive.PathToPose(new Pose2d(new Translation2d(12, 2), new Rotation2d(11*Math.PI/6)), 0)
    // );

    return m_drive.goToPose(new Pose2d(new Translation2d(4, 6), new Rotation2d(3*Math.PI/4))).andThen(
      m_drive.goToPose(new Pose2d(new Translation2d(12, 2), new Rotation2d(11*Math.PI/6)))
    );
  
  }
}
