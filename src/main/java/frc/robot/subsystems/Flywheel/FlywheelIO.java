package frc.robot.subsystems.Flywheel;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public interface FlywheelIO extends Subsystem {
  void set(double speed);

  void setVelocity(double rpm);

  void setVoltage(double volts);

  void setEncoderPosition(double position);
  
  double getPosition();

  double getVelocity();

  boolean atTarget(double threshold);

  boolean atRealTarget(double threshold);

  void setRealTarget(double rpm);

  SubsystemBase returnSubsystem();

  double getVoltage();
}
