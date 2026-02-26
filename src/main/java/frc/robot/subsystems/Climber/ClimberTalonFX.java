package frc.robot.subsystems.Climber;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ClimberConstants;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import org.littletonrobotics.junction.Logger;

public class ClimberTalonFX extends SubsystemBase implements ClimberIO {

	private final TalonFX climberMotor = new TalonFX(ClimberConstants.kClimberCanID);

	private static final double kRotationsPerInch = 1.0 / ClimberConstants.kInchesPerMotorRotation;

	public ClimberTalonFX() {
		climberMotor.getConfigurator().apply(Configs.Climber.ClimberConfig);
	}

	private double targetRots = 0.0;

	@Override
	public void periodic() {
		SmartDashboard.putNumber("Climber Position (Rots)", climberMotor.getPosition().getValue().in(Rotations));
		SmartDashboard.putNumber("Climber Extension (in)", climberMotor.getPosition().getValueAsDouble() * ClimberConstants.kInchesPerMotorRotation);
		Logger.recordOutput("FinalComponentPoses/Climber Position", new Pose3d(0,0,getPosition().in(Meters), new Rotation3d()));
	}

	@Override
	public void set(double speed) {
		climberMotor.set(speed);
	}

	@Override
	public void setPosition(double rots) {
		SmartDashboard.putNumber("Requested Climber Position (rots)", rots);
		climberMotor.setControl(new PositionVoltage(rots).withSlot(0));
		targetRots = rots;
	}

	@Override
	public void setVoltage(double volts) {
		climberMotor.setControl(new VoltageOut(volts));
	}

	@Override
	public void setEncoderPosition(double rotations) {
		climberMotor.setPosition(rotations);
	}

	@Override
	public Distance getPosition() {
		double rotations = climberMotor.getPosition().getValueAsDouble();
		double inches = rotations / kRotationsPerInch;
		double meters = Units.inchesToMeters(inches);
		return Meters.of(meters);
	}

	@Override
	public boolean atTarget(double threshold) {
		return Math.abs(climberMotor.getPosition().getValueAsDouble() - targetRots) < threshold;
	}

	@Override
	public SubsystemBase returnSubsystem() {
		return this;
	}

	@Override
	public double getStatorCurrent() {
		return climberMotor.getStatorCurrent().getValueAsDouble();
	}

	public void stop() {
		climberMotor.setControl(new NeutralOut());
	}
}
