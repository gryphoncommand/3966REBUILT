package frc.robot.subsystems.Flywheel;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ShooterConstants;

public class FlywheelSparkFlex extends SubsystemBase implements FlywheelIO {
  private final SparkFlex shooterMotor = new SparkFlex(ShooterConstants.kFlywheelCanID, MotorType.kBrushless);
  private final SparkFlex followerMotor = new SparkFlex(ShooterConstants.kFollowerWheelCanID, MotorType.kBrushless);
  private final SparkClosedLoopController pid = shooterMotor.getClosedLoopController();
  private final RelativeEncoder encoder = shooterMotor.getEncoder();

  private double targetReference = 0;
  private ControlType currentControlType = ControlType.kPosition;

  public FlywheelSparkFlex() {
    shooterMotor.configure(Configs.FlywheelConfig.flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    followerMotor.configure(Configs.FlywheelConfig.flywheelConfig.follow(ShooterConstants.kFlywheelCanID, true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Velocity (RPM)", getVelocity());
    SmartDashboard.putNumber("Desired Flywheel Speed", pid.getSetpoint());
    Logger.recordOutput("Flywheel Applied Output", shooterMotor.getAppliedOutput());
  }

  @Override
  public void set(double speed) {
    shooterMotor.set(speed);
  }

  @Override
  public void setVelocity(double rpm) {
    targetReference = rpm;
    pid.setSetpoint(rpm, ControlType.kVelocity);
    currentControlType = ControlType.kVelocity;
  }

  public void setPosition(double position){
    pid.setSetpoint(position, ControlType.kPosition);
    
    targetReference = position;
    currentControlType = ControlType.kPosition;
}

  @Override
  public void setVoltage(double volts) {
    shooterMotor.setVoltage(volts);
  }

  @Override
  public double getPosition() {
      return encoder.getPosition();
  }

  @Override
  public void setEncoderPosition(double position) {
    encoder.setPosition(position);
  }

  @Override
  public double getVelocity() {
    return encoder.getVelocity();
  }

  @Override
  public boolean atTarget(double threshold) {
    if (currentControlType == ControlType.kVelocity) {
        return Math.abs(getVelocity() - targetReference) < threshold;
    } else {
        return false;
    }
  }

  @Override
  public SubsystemBase returnSubsystem() {
    return this;
  }

  @Override
  public double getVoltage() {
      return shooterMotor.get() * RobotController.getBatteryVoltage();
  }
}
