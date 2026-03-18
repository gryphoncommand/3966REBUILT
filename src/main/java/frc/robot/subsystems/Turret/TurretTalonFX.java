package frc.robot.subsystems.Turret;

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
import frc.robot.Constants.TurretConstants;

public class TurretTalonFX extends SubsystemBase implements TurretIO {

  private final TalonFX turretMotor =
      new TalonFX(TurretConstants.kTurretCanID);

  private final PositionVoltage positionRequest = new PositionVoltage(0);

  private double targetAngleDeg = TurretConstants.kTurretHomeAngleDeg;

  // rotations per turret degree (depends on gearing)
  private static final double kRotationsPerDegree =
      TurretConstants.kTurretGearRatio / 360.0;

  public TurretTalonFX() {
    turretMotor.getConfigurator().apply(Configs.Turret.TurretConfig);
    turretMotor.setPosition(targetAngleDeg * kRotationsPerDegree);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Turret/Turret Angle (deg)", getAngle());
    Logger.recordOutput("Turret/Desired Turret Angle", targetAngleDeg);
    Logger.recordOutput("Turret/Applied Turret Volts", turretMotor.getMotorVoltage().getValue().in(Volts));
    Logger.recordOutput(
        "FinalComponentPoses/Turret Position",
        new Pose3d(
            ShooterConstants.kRobotToShooter.getX(),
            ShooterConstants.kRobotToShooter.getY(),
            TurretConstants.kTurretHeightMeters,
            new Rotation3d(0.0, 0.0, Units.degreesToRadians(getAngle()))
        )
    );
  }

  @Override
  public void set(double speed) {
    setVoltage(speed * turretMotor.getSupplyVoltage().getValue().in(Volts));
  }

  @Override
  public void setAngle(double degrees) {
    targetAngleDeg = degrees;
    SmartDashboard.putNumber("Requested Turret Position", degrees);
    turretMotor.setControl(positionRequest.withPosition(degrees * kRotationsPerDegree));
  }

  @Override
  public void setVoltage(double volts) {
    turretMotor.setVoltage(volts);
  }

  @Override
  public void setEncoderPosition(double degrees) {
    turretMotor.setPosition(degrees * kRotationsPerDegree);
  }

  @Override
  public double getAngle() {
    return turretMotor.getPosition().getValueAsDouble() / kRotationsPerDegree;
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
      return turretMotor.getStatorCurrent().getValueAsDouble();
  }

  @Override
  public void stop() {
      turretMotor.setControl(new NeutralOut());
  }
}
