package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hood.HoodIO;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Flywheel.FlywheelIO;
import frc.GryphonLib.ShooterState;

public class SetShooterToDefinedState extends Command {

  private final HoodIO hood;
  private final FlywheelIO flywheel;
  private final ShooterState state;

  public SetShooterToDefinedState(
      HoodIO hood,
      FlywheelIO flywheel,
      ShooterState state) {

    this.hood = hood;
    this.flywheel = flywheel;
    this.state = state;

    addRequirements(hood.returnSubsystem(),
                    flywheel.returnSubsystem());
  }

  @Override
  public void execute() {
    double rpm = state.flywheelRPM();
    if(rpm > flywheel.getVelocity() + 100){
      rpm += ShooterConstants.kFlywheelRPMOffset;
    }
    hood.setAngle(state.hoodAngleDeg());
    flywheel.setVelocity(rpm);
  }

  @Override
  public boolean isFinished() {
    return false;
    // return hood.atTarget(1.0) && flywheel.atTarget(30);
  }

  @Override
  public void end(boolean interrupted) {
      flywheel.setVelocity(0);
      flywheel.set(0);
  }
}
