package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hood.HoodIO;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Flywheel.FlywheelIO;
import frc.GryphonLib.ShooterState;

public class SetShooterToDefinedState extends Command {

  private final HoodIO hood;
  private final FlywheelIO flywheel;
  private final ShooterState state;
  private boolean reachedTarget;
  private Timer rechargeTimer = new Timer();

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
  public void initialize() {
    reachedTarget = false;
    rechargeTimer.restart();
  }

  @Override
  public void execute() {
    double rpm = state.flywheelRPM();
    flywheel.setRealTarget(rpm);
    if(rpm > flywheel.getVelocity() + 100){
      rpm += ShooterConstants.kFlywheelRPMOffset;
    }
    hood.setAngle(state.hoodAngleDeg());
    flywheel.setVelocity(rpm);

    if (Math.abs(state.flywheelRPM() - flywheel.getVelocity()) < 100 && reachedTarget == false){
        reachedTarget = true;
        Logger.recordOutput("Recharge Time", String.valueOf(rechargeTimer.get()));
    } else {
        if (!(Math.abs(state.flywheelRPM() - flywheel.getVelocity()) < 100) && reachedTarget == true){
            reachedTarget = false;
            rechargeTimer.reset();
        }
    }
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
