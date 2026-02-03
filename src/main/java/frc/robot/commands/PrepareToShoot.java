package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hood.HoodIO;
import frc.robot.subsystems.Flywheel.FlywheelIO;
import frc.GryphonLib.ShooterState;
import frc.GryphonLib.ShooterInterpolator;

import java.util.List;
import java.util.function.DoubleSupplier;

public class PrepareToShoot extends Command {

  private final HoodIO hood;
  private final FlywheelIO flywheel;
  private final DoubleSupplier distanceSupplier;
  private final List<ShooterState> table;

  public PrepareToShoot(
      HoodIO hood,
      FlywheelIO flywheel,
      DoubleSupplier distanceSupplier,
      List<ShooterState> table) {

    this.hood = hood;
    this.flywheel = flywheel;
    this.distanceSupplier = distanceSupplier;
    this.table = table;

    addRequirements(hood.returnSubsystem(),
                    flywheel.returnSubsystem());
  }

  @Override
  public void execute() {
    ShooterState state =
        ShooterInterpolator.interpolate(
            table, distanceSupplier.getAsDouble());

    hood.setAngle(state.hoodAngleDeg());
    flywheel.setVelocity(state.flywheelRPM());
  }

  @Override
  public boolean isFinished() {
    return hood.atTarget(3.0) && flywheel.atTarget(50);
  }
}
