package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake.IntakeDeployIO;

public class HomeIntake extends Command {
  private final IntakeDeployIO intakeDeploy;
  private final Timer timer = new Timer();

  public HomeIntake(IntakeDeployIO intakeDeploy) {
    this.intakeDeploy = intakeDeploy;
    addRequirements(intakeDeploy);
  }

  @Override
  public void initialize() {
    timer.restart();
    // Drive slowly downwards
    intakeDeploy.setVoltage(-2.4);
  }

  @Override
  public void execute() {
    SmartDashboard.putNumber("Hood Homing Stator Current", intakeDeploy.getStatorCurrent());
  }

  @Override
  public boolean isFinished() {
    // Wait 0.5s for the motor to start moving, then check if current > 20 Amps
    return timer.get() > 0.5 && intakeDeploy.getStatorCurrent() > 7.5;
  }

  @Override
  public void end(boolean interrupted) {
    intakeDeploy.set(0);
    if (!interrupted) {
      // Once we hit the bottom, tell the Talon this is exactly MinAngle
      intakeDeploy.setEncoderPosition(IntakeConstants.kIntakeDeployAngle);
    }
  }
}