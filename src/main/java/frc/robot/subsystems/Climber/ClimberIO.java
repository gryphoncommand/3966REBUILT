package frc.robot.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public interface ClimberIO extends Subsystem {

  /** Percent output (-1 to 1) */
  void set(double speed);

  /** Set climber position */
  void setPosition(double rots);

  /** Direct voltage control */
  void setVoltage(double volts);

  /** Reset encoder position (rotations) */
  void setEncoderPosition(double rotations);

  /** Current climber position */
  double getPosition();

  /** True if within threshold of target */
  boolean atTarget(double threshold);

  SubsystemBase returnSubsystem();

  double getStatorCurrent();
}
