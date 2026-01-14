package frc.robot.subsystems.Hood;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ShooterConstants;

public class HoodTalonFX extends SubsystemBase implements HoodIO {

  private final TalonFX hoodMotor =
      new TalonFX(ShooterConstants.kHoodCANID);

  private final PositionVoltage positionRequest = new PositionVoltage(0);

  private double targetAngleDeg = 0.0;
  private double startingValue = ShooterConstants.kHoodMinAngleDeg;

  // rotations per hood degree (depends on gearing)
  private static final double kRotationsPerDegree =
      ShooterConstants.kHoodGearRatio / 360.0;

  public HoodTalonFX() {
    hoodMotor.getConfigurator().apply(Configs.Hood.HoodConfig);
  }

  @Override
  public void periodic() {
    hoodMotor.setPosition(startingValue * kRotationsPerDegree);
    SmartDashboard.putNumber("Hood Angle (deg)", getAngle());
    SmartDashboard.putNumber("Hood Target (deg)", targetAngleDeg);
  }

  @Override
  public void set(double speed) {
    hoodMotor.set(speed);
  }

  @Override
  public void setAngle(double degrees) {
    targetAngleDeg = degrees;
    hoodMotor.setControl(positionRequest.withPosition(degrees * kRotationsPerDegree));
  }

  @Override
  public void setVoltage(double volts) {
    hoodMotor.setVoltage(volts);
  }

  @Override
  public void setEncoderPosition(double degrees) {
    hoodMotor.setPosition(degrees * kRotationsPerDegree);
  }

  @Override
  public double getAngle() {
    return hoodMotor.getPosition().getValueAsDouble() / kRotationsPerDegree;
  }

  @Override
  public boolean atTarget(double thresholdDeg) {
    return Math.abs(getAngle() - targetAngleDeg) < thresholdDeg;
  }

  @Override
  public SubsystemBase returnSubsystem() {
    return this;
  }
}
