package frc.robot.subsystems.Flywheel;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ShooterConstants;

/**
 * Hardware-backed Flywheel implementation using a TalonFX.
 * Mirrors the public behavior of the simulation class but delegates
 * physics to the real motor controller / hardware.
 */
public class FlywheelTalonFX extends SubsystemBase implements FlywheelIO {
    private final TalonFX m_flywheelMotor;

    private double targetVelocityRpm = 0;
    private double realTarget = 0;

    private final Mechanism2d mech2d = new Mechanism2d(0.5, 0.5); // 0.5 m square
    private final MechanismRoot2d root = mech2d.getRoot("ShooterBase", 0.25, 0.25);
    private final MechanismLigament2d wheelVisual = root.append(new MechanismLigament2d("Wheel", 0.05, 0));

    public FlywheelTalonFX() {
        wheelVisual.setLineWeight(2);
        m_flywheelMotor = new TalonFX(ShooterConstants.kFlywheelCanID);
        m_flywheelMotor.getConfigurator().apply(Configs.FlywheelConfig.flywheelFXConfig);
        m_flywheelMotor.getVelocity().setUpdateFrequency(1000);
    }

    @Override
    public void periodic() {
        SmartDashboard.putData("Shooter Mech", mech2d);
        Logger.recordOutput("Flywheel/Flywheel Motor Velocity (RPM)", m_flywheelMotor.getVelocity().getValue().in(RPM)/ShooterConstants.kGearRatio);
        Logger.recordOutput("Flywheel/Flywheel Velocity (RPM)", getVelocity());
        Logger.recordOutput("Flywheel/Effective Desired Flywheel Speed", targetVelocityRpm);
        Logger.recordOutput("Flywheel/Desired Flywheel Speed", realTarget);
        Logger.recordOutput("Flywheel/Flywheel Applied Output (Duty Cycle)", m_flywheelMotor.get());
        Logger.recordOutput("Flywheel/Flywheel Applied Output (Volts)", getVoltage());
    }

    @Override
    public void set(double speed) {
        m_flywheelMotor.set(speed);
    }

    @Override
    public void setVelocity(double rpm) {
        targetVelocityRpm = rpm;
        double rotorRps = RPM.of(rpm).in(RotationsPerSecond);
        m_flywheelMotor.setControl(new VelocityVoltage(rotorRps).withEnableFOC(true));
    }

    @Override
    public void setRealTarget(double rpm) {
        realTarget = rpm;
    }

    @Override
    public boolean atRealTarget(double threshold) {
        return Math.abs(getVelocity() - realTarget) < threshold;
    }

    @Override
    public void setVoltage(double volts) {
        m_flywheelMotor.setVoltage(volts);
    }

    @Override
    public double getPosition() {
        // Return mechanism rotations (wheel rotations)
        return m_flywheelMotor.getPosition().getValue().in(Rotations);
    }

    @Override
    public void setEncoderPosition(double position) {
        // position is mechanism rotations; set rotor rotations accordingly
        m_flywheelMotor.setPosition(position * ShooterConstants.kGearRatio);
    }

    @Override
    public double getVelocity() {
        // motor velocity is rotor rpm; convert to mechanism rpm
        return m_flywheelMotor.getVelocity().getValue().in(RPM);
    }

    @Override
    public boolean atTarget(double threshold) {
        return Math.abs(getVelocity() - targetVelocityRpm) < threshold;
    }

    @Override
    public SubsystemBase returnSubsystem() {
        return this;
    }

    @Override
    public double getVoltage() {
        return m_flywheelMotor.getMotorVoltage().getValue().in(Volts);
    }

    @Override
    public double getStatorCurrent() {
        return m_flywheelMotor.getStatorCurrent().getValue().in(Amps);
    }

    @Override
    public void stop() {
        setVelocity(0);
        setRealTarget(0);
        set(0);
    }
}