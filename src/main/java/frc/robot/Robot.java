// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.littletonUtils.HubShiftUtil;
import frc.robot.Constants.AlignmentConstants;
import frc.FuelSim;
import frc.GryphonLib.AllianceFlipUtil;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot  {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private boolean gotAlliance = false;

  public Robot(){
    Logger.addDataReceiver(new NT4Publisher());
    Logger.addDataReceiver(new WPILOGWriter());
    Logger.start();
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    DataLogManager.start();
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());
    gotAlliance = false;
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    if (DriverStation.getAlliance().isPresent() && !gotAlliance){
      AlignmentConstants.HubPose = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red ? AlignmentConstants.RedHubPose : AlignmentConstants.BlueHubPose;
      AlignmentConstants.PassingPoseOutpost = (new Pose2d(AllianceFlipUtil.applyX(0.812), 2.2688, new Rotation2d()));
      AlignmentConstants.PassingPoseDepot = (new Pose2d(AllianceFlipUtil.applyX(0.812), 5.707, new Rotation2d()));
      gotAlliance = true;
    }
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    SmartDashboard.putData(CommandScheduler.getInstance());
    if (RobotController.getBatteryVoltage() < 7.5){
      Logger.recordOutput("Battery/Browned Out", true);
    }
    Logger.recordOutput("Basics/Match Time", DriverStation.getMatchTime());
    Logger.recordOutput("Basics/Recieving Alliance", DriverStation.getAlliance().isPresent());
    Logger.recordOutput("Basics/Current Alliance", DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get().toString() : "Neither");
    Logger.recordOutput("Basics/Hub State", HubShiftUtil.getShiftedShiftInfo().active());
    Logger.recordOutput("Basics/OpposingHub State", HubShiftUtil.isOpposingHubActive());
    Logger.recordOutput("Basics/Remaining Time Until Shift", HubShiftUtil.getShiftedShiftInfo().remainingTime());
  }

  @Override
  public void simulationPeriodic() {
    FuelSim.getInstance().updateSim();
  }

  @Override
  public void simulationInit() {}

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    AlignmentConstants.HubPose = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red ? AlignmentConstants.RedHubPose : AlignmentConstants.BlueHubPose;
    AlignmentConstants.PassingPoseOutpost = (new Pose2d(AllianceFlipUtil.applyX(1.012), 2.2688, new Rotation2d()));
    AlignmentConstants.PassingPoseDepot = (new Pose2d(AllianceFlipUtil.applyX(1.012), 5.707, new Rotation2d()));
    HubShiftUtil.initialize();
    CommandScheduler.getInstance().schedule(Commands.runOnce(() -> {
          FuelSim.getInstance().clearFuel();
          FuelSim.getInstance().spawnStartingFuel();
          FuelSim.Hub.BLUE_HUB.resetScore();
          FuelSim.Hub.RED_HUB.resetScore();
          if (Robot.isSimulation()) {
            FuelSim.getInstance().reserveFuelForRobot(m_robotContainer.getSimBallCount());
          }
      })
      .withName("Reset Fuel")
      .ignoringDisable(true));
    
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    Logger.recordOutput("Battery/Browned Out", false);
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
  
}
