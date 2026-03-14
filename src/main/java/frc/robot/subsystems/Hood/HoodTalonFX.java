package frc.robot.subsystems.Hood;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
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
    hoodMotor.setPosition(startingValue * kRotationsPerDegree);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Hood/Hood Angle (deg)", getAngle());
    Logger.recordOutput("Hood/Desired Hood Angle", targetAngleDeg);
    Logger.recordOutput("Hood/Applied Hood Volts", hoodMotor.getMotorVoltage().getValue().in(Volts));
    Logger.recordOutput("FinalComponentPoses/Hood Position", new Pose3d(0, 0.09, 0.41, new Rotation3d(Units.degreesToRadians(-getAngle() + ShooterConstants.kHoodMinAngleDeg), 0.0, 0.0)));
  }

  @Override
  public void set(double speed) {
    setVoltage(speed * hoodMotor.getSupplyVoltage().getValue().in(Volts));
  }

  @Override
  public void setAngle(double degrees) {
    targetAngleDeg = degrees;
    SmartDashboard.putNumber("Requested Hood Position", degrees);
    hoodMotor.setControl(positionRequest.withPosition(degrees * kRotationsPerDegree));
  }

  @Override
  public void stow() {
    SmartDashboard.putNumber("Requested Hood Position", ShooterConstants.kHoodMinAngleDeg);
    hoodMotor.setControl(positionRequest.withPosition(ShooterConstants.kHoodMinAngleDeg * kRotationsPerDegree));
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

  @Override
  public double getStatorCurrent() {
      return hoodMotor.getStatorCurrent().getValueAsDouble();
  }

  @Override
  public void stop() {
      hoodMotor.setControl(new NeutralOut());
  }
}
