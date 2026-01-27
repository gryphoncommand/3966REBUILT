package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Hood.HoodIO;

public class HomeHood extends Command {
  private final HoodIO hood;
  private final Timer timer = new Timer();

  public HomeHood(HoodIO hood) {
    this.hood = hood;
    addRequirements(hood);
  }

  @Override
  public void initialize() {
    timer.restart();
    // Drive slowly downwards
    hood.setVoltage(-1.0); 
  }

  @Override
  public void execute() {
    SmartDashboard.putNumber("Hood Homing Stator Current", hood.getStatorCurrent());
  }

  @Override
  public boolean isFinished() {
    // Wait 0.5s for the motor to start moving, then check if current > 20 Amps
    return timer.get() > 0.5 && hood.getStatorCurrent() > 20.0;
  }

  @Override
  public void end(boolean interrupted) {
    hood.stop();
    if (!interrupted) {
      // Once we hit the bottom, tell the Talon this is exactly MinAngle
      hood.setEncoderPosition(ShooterConstants.kHoodMinAngleDeg);
    }
  }
}