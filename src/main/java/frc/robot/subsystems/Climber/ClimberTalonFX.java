package frc.robot.subsystems.Climber;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ClimberConstants;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

public class ClimberTalonFX extends SubsystemBase implements ClimberIO {

	private final TalonFX climberMotor = new TalonFX(ClimberConstants.kClimberCanID);

	public ClimberTalonFX() {
		climberMotor.getConfigurator().apply(Configs.Climber.ClimberConfig);
	}

	private double targetRots = 0.0;

	@Override
	public void periodic() {
		Logger.recordOutput("Climber/Climber Height (in)", climberMotor.getPosition().getValue().in(Rotations) * ClimberConstants.kInchesPerMotorRotation);
        Logger.recordOutput("Climber/Climber Motor Rots", climberMotor.getPosition().getValue().in(Rotations));
        Logger.recordOutput("Climber/Applied Volts", climberMotor.getMotorVoltage().getValue().in(Volts));
		Logger.recordOutput("FinalComponentPoses/Climber Position", new Pose3d(0,0,Units.inchesToMeters(climberMotor.getPosition().getValueAsDouble() * ClimberConstants.kInchesPerMotorRotation), new Rotation3d()));
	}

	@Override
	public void set(double speed) {
		climberMotor.set(speed);
	}

	@Override
	public void setPosition(double rots) {
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
	public double getPosition() {
		return climberMotor.getPosition().getValue().in(Rotations);
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
