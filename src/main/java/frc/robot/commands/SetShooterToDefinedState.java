package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hood.HoodIO;
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
    
    hood.setAngle(state.hoodAngleDeg);
    flywheel.setVelocity(state.flywheelRPM);
  }

  @Override
  public boolean isFinished() {
    return hood.atTarget(3.0) && flywheel.atTarget(50);
  }
}
