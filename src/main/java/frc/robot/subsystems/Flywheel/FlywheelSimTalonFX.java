package frc.robot.subsystems.Flywheel;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class FlywheelSimTalonFX extends SubsystemBase implements FlywheelIO {
    private final FlywheelSim shooterSim =
        new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNeoVortex(3), 0.04, 1), DCMotor.getNeoVortex(3), 0.005);

    private double appliedVolts = 0;
    private double targetVelocityRpm = 0;
    private double wheelAngle = 0.0;

    private final Mechanism2d mech2d = new Mechanism2d(0.5, 0.5); // 0.5 m square
    private final MechanismRoot2d root = mech2d.getRoot("ShooterBase", 0.25, 0.25);
    private final MechanismLigament2d wheelVisual = root.append(new MechanismLigament2d("Wheel", 0.2, 0));

    private final TalonFX m_flywheelMotor;
    private final TalonFXSimState m_flywheelSim;

    final VelocityVoltage m_flywheelVelocityVoltage = new VelocityVoltage(0);
    

    public FlywheelSimTalonFX() {
        m_flywheelMotor = new TalonFX(ShooterConstants.kFlywheelCANID);
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
    wheelAngle %= 360;
    wheelVisual.setAngle(wheelAngle);

    SmartDashboard.putData("Shooter Mech", mech2d);
    SmartDashboard.putNumber("Shooter Velocity (RPM)", mechVel.in(RPM));
    SmartDashboard.putNumber("Shooter Applied Volts", appliedVolts);
    }

    @Override
    public void set(double speed) {
        appliedVolts = speed * RobotController.getBatteryVoltage();
    }

    @Override
    public void setVelocity(double rpm) {
        // In sim we approximate by setting voltage proportional to error
        targetVelocityRpm = rpm;
        double error = rpm - getVelocity();
        appliedVolts = Math.max(-12, Math.min(12, error * 0.1)); // crude kP control
    }

    @Override
    public void setVoltage(double volts) {
        appliedVolts = volts;
    }

    @Override
    public void setEncoderPosition(double position) {
        shooterSim.setState(VecBuilder.fill(position));
    }

    @Override
    public double getVelocity() {
        return shooterSim.getAngularVelocityRPM();
    }


    @Override
    public boolean atTarget(double threshold) {
        return Math.abs(getVelocity() - targetVelocityRpm) < threshold;
    }

    @Override
    public SubsystemBase returnSubsystem() {
        return this;
    }
}
