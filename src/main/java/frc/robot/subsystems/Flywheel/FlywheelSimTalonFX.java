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
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(ShooterConstants.kDrumMotorCount), Jkgm2, 1),
            DCMotor.getKrakenX60(ShooterConstants.kDrumMotorCount),
            0.2);

    private double targetVelocityRpm = 0;
    private double wheelAngle = 0.0;

    private final Mechanism2d mech2d = new Mechanism2d(0.5, 0.5); // 0.5 m square
    private final MechanismRoot2d root = mech2d.getRoot("ShooterBase", 0.25, 0.25);
    private final MechanismLigament2d wheelVisual = root.append(new MechanismLigament2d("Wheel", 0.05, 0));

    private final TalonFX m_flywheelMotor;
    private final TalonFXSimState m_flywheelSim;

    private double realTarget = 0;

    private double pendingShotTorqueNm = 0;
    private double shotTimeRemainingSec = 0;
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
        var motorVoltageTurn = m_flywheelSim.getMotorVoltageMeasure();
        double dt = 0.02; // 20ms loop

        double appliedLoadTorque = 0;
        if (shotTimeRemainingSec > 0) {
            // We apply the torque stored in pendingShotTorqueNm
            appliedLoadTorque = pendingShotTorqueNm;
            
            // Reduce the remaining time
            shotTimeRemainingSec -= dt;
            
            // If time is up, clear the torque buffer
            if (shotTimeRemainingSec <= 0) {
                shotTimeRemainingSec = 0;
                pendingShotTorqueNm = 0;
            }
        }

        // Update physics
        shooterSim.setInputVoltage(motorVoltageTurn.in(Volts));
        shooterSim.update(dt);
        
        double deltaOmegaB = 0;

        // Apply the deceleration from the ball(s)
        if (appliedLoadTorque > 0) {
            double alpha = appliedLoadTorque / Jkgm2;
            deltaOmegaB = (alpha * dt);
        }

        // Current angular velocity
        double omega = shooterSim.getAngularVelocity().in(RadiansPerSecond);

        // --- Friction torque ---
        double coulomb = ShooterConstants.kCoulombFrictionNm * Math.signum(omega);
        double viscous = ShooterConstants.kViscousFriction * omega;

        // Total opposing torque
        double frictionTorque = coulomb + viscous;

        // Angular deceleration
        double alpha = frictionTorque / Jkgm2;

        // Apply velocity drop
        double deltaOmegaT = alpha * dt;

        // Prevent oscillation around zero
        if (Math.abs(deltaOmegaT) < 1e-3) {
            deltaOmegaT = 0;
        }

        shooterSim.setAngularVelocity(omega - deltaOmegaB - deltaOmegaT);

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
        ballExitVelocityMps *= 8/5; // Backspin
        double ballMassKg = 0.226796; 
        double currentOmega = shooterSim.getAngularVelocity().in(RadiansPerSecond);

        if (currentOmega < 10.0) return; // Flywheel is basically stopped

        // Energy required for ONE ball: E = 1/2 * m * v^2
        double ballEnergyJ = 0.5 * ballMassKg * Math.pow(ballExitVelocityMps, 2);
        
        // Time one ball spends in contact with the wheel
        double singleBallContactTime = 0.05; 

        // Torque for one ball: T = (Energy / Time) / Velocity
        double singleBallTorque = (ballEnergyJ / singleBallContactTime) / currentOmega;

        // ADD to the current simulation state rather than overwriting it
        // This allows "stacking" shots
        this.pendingShotTorqueNm += (singleBallTorque * torqueRandomizer.nextDouble(1.4, 2.8)); // 1.4 for friction/compression losses
        this.shotTimeRemainingSec += singleBallContactTime; 
    }

    @Override
    public void stop() {
        setVelocity(0);
        setRealTarget(0);
        set(0);
    }
}
