package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public interface IntakeDeployIO extends Subsystem {

  /** Percent output (-1 to 1) */
  void set(double speed);

  /** Set hood angle in rotations */
  void setPosition(double position);

  /** Direct voltage control */
  void setVoltage(double volts);

  /** Set velocity in rpm */
  void setVelocity(double rpm);

  /** Current velocity in rpm */
  double getVelocity();

  /** Reset encoder position (rotations) */
  void setEncoderPosition(double rotations);

  /** Current position in rotations */
  double getPosition();

  /** True if within threshold (rotations) of target */
  boolean atTarget(double threshold);

  SubsystemBase returnSubsystem();

  double getStatorCurrent();
}
