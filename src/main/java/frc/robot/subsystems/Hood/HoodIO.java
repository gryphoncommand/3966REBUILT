package frc.robot.subsystems.Hood;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public interface HoodIO extends Subsystem {

  /** Percent output (-1 to 1) */
  void set(double speed);

  /** Set hood angle in degrees */
  void setAngle(double degrees);

  /** Direct voltage control */
  void setVoltage(double volts);

  /** Reset encoder position (degrees) */
  void setEncoderPosition(double degrees);

  /** Current hood angle in degrees */
  double getAngle();

  /** True if within threshold (deg) of target */
  boolean atTarget(double thresholdDeg);

  SubsystemBase returnSubsystem();
}
