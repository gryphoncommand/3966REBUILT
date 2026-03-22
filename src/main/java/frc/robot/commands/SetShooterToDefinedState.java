package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Flywheel.FlywheelIO;
import frc.GryphonLib.ShooterState;

public class SetShooterToDefinedState extends Command {

  private final FlywheelIO flywheel;
  private final ShooterState state;
  private boolean reachedTarget;
  private Timer rechargeTimer = new Timer();

  public SetShooterToDefinedState(
      FlywheelIO flywheel,
      ShooterState state) {

    this.flywheel = flywheel;
    this.state = state;
    

    addRequirements(flywheel.returnSubsystem());
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
    if(!(flywheel.atRealTarget(100)) && rpm > flywheel.getVelocity()){
      rpm += ShooterConstants.kFlywheelRPMOffset;
    }
    flywheel.setVelocity(rpm);

    if (flywheel.atRealTarget(100) && reachedTarget == false){
        reachedTarget = true;
        Logger.recordOutput("Recharge Time", String.valueOf(rechargeTimer.get()));
    } else {
        if (!(flywheel.atRealTarget(100)) && reachedTarget == true){
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
      flywheel.setRealTarget(0);
      flywheel.setVelocity(0);
      flywheel.set(0);
  }
}
