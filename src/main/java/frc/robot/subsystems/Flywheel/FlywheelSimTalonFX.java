package frc.robot.subsystems.Flywheel;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.Random;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ShooterConstants;

public class FlywheelSimTalonFX extends SubsystemBase implements FlywheelIO {
    private double Jkgm2 = 0.01;
    private final FlywheelSim shooterSim =
        new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNeoVortex(2), Jkgm2, 1), DCMotor.getNeoVortex(2), 0.0);

    private double targetVelocityRpm = 0;
    private double wheelAngle = 0.0;

    private final Mechanism2d mech2d = new Mechanism2d(0.5, 0.5); // 0.5 m square
    private final MechanismRoot2d root = mech2d.getRoot("ShooterBase", 0.25, 0.25);
    private final MechanismLigament2d wheelVisual = root.append(new MechanismLigament2d("Wheel", 0.05, 0));

    private final TalonFX m_flywheelMotor;
    private final TalonFXSimState m_flywheelSim;

    private double realTarget = 0;

    private Random torqueRandomizer = new Random();

    final VelocityVoltage m_flywheelVelocityVoltage = new VelocityVoltage(0);
    

    public FlywheelSimTalonFX() {
        wheelVisual.setLineWeight(2);
        m_flywheelMotor = new TalonFX(ShooterConstants.kFlywheelCanID);
        m_flywheelMotor.getConfigurator().apply(Configs.FlywheelConfig.flywheelFXConfig);
        m_flywheelSim = m_flywheelMotor.getSimState();
    }

    @Override
    public void periodic() {
        // get the motor voltage of the TalonFX
        var motorVoltageTurn = m_flywheelSim.getMotorVoltageMeasure();

        // use the motor voltage to calculate new position and velocity
        // using WPILib's DCMotorSim class for physics simulation
        shooterSim.setInputVoltage(motorVoltageTurn.in(Volts));
        shooterSim.update(0.02);

        // apply the new rotor position and velocity to the TalonFX;
        // note that this is rotor position/velocity (before gear ratio), but
        // DCMotorSim returns mechanism position/velocity (after gear ratio)
        AngularVelocity mechVel = shooterSim.getAngularVelocity();

        m_flywheelSim.setRotorVelocity(mechVel.in(RotationsPerSecond));

        wheelAngle += Units.rotationsToDegrees(mechVel.in(RPM) * 0.02 / 60.0);
        wheelVisual.setAngle(wheelAngle);

        SmartDashboard.putData("Shooter Mech", mech2d);
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
        m_flywheelMotor.setControl(new VelocityVoltage(RPM.of(rpm).in(RotationsPerSecond)));
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
        return m_flywheelMotor.getPosition().getValue().in(Rotations);
    }

    @Override
    public void setEncoderPosition(double position) {
        m_flywheelMotor.setPosition(position);
        shooterSim.setState(VecBuilder.fill(position));
    }

    @Override
    public double getVelocity() {
        return shooterSim.getAngularVelocity().in(RPM);
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

    public void simulateShot(double ballExitVelocityMps) {
        double ballMassKg = 0.226796;
        // Flywheel parameters
        double flywheelRadiusM = 0.0508;
        double J = Jkgm2;
        
        // Angular momentum removed by the ball
        double deltaL = ballMassKg * flywheelRadiusM * ballExitVelocityMps;
        
        // Convert to angular velocity drop (rad/s)
        double deltaOmegaRadPerSec = deltaL / J;
        
        // Current flywheel velocity (rad/s)
        double currentOmega = shooterSim.getAngularVelocity().in(RadiansPerSecond);
        
        // Apply realistic velocity drop
        double newOmega = currentOmega - (deltaOmegaRadPerSec * torqueRandomizer.nextDouble(1.2, 1.8));
        shooterSim.setAngularVelocity(newOmega);
        
        // Update TalonFX sim rotor velocity to match new flywheel state
        m_flywheelSim.setRotorVelocity(RotationsPerSecond.of(newOmega / (2*Math.PI)));
    }

    @Override
    public void stop() {
        setVelocity(0);
        setRealTarget(0);
        set(0);
    }
}
