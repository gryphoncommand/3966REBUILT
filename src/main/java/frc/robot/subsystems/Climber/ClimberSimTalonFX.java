package frc.robot.subsystems.Climber;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Configs;
import frc.robot.Constants.ClimberConstants;

public class ClimberSimTalonFX extends SubsystemBase implements ClimberIO {

    private final TalonFX climberMotor = new TalonFX(ClimberConstants.kClimberCanID);
    private final TalonFXSimState climberSimState = climberMotor.getSimState();

    // Moment of inertia for the spool (arbitrary)
    private final double jKgMS = 10;

    // Model climber as a single-jointed arm sim
    private final SingleJointedArmSim climberSim = new SingleJointedArmSim(
        DCMotor.getKrakenX60(1),
        ClimberConstants.kGearRatio,
        jKgMS,
        1.0, // unit length in meters, will scale
        0,   // min position
        Units.inchesToMeters(10), // max climber height
        false,
        0
    );

    private Distance targetPosition = Inches.of(0);

    private final Mechanism2d mech2d = new Mechanism2d(0.2, 0.6);
    private final MechanismRoot2d root = mech2d.getRoot("climberRoot", 0.1, 0.0);
    private final MechanismLigament2d climberVisual =
        root.append(new MechanismLigament2d("climber", 0.0, 0, 10, new Color8Bit(Color.kRed)));

    public ClimberSimTalonFX() {
        climberMotor.getConfigurator().apply(Configs.Climber.ClimberConfig);
        double rotorPos = climberSim.getAngleRads() / (2 * Math.PI) * ClimberConstants.kGearRatio;
        climberSimState.setRawRotorPosition(rotorPos);
    }

    @Override
    public void periodic() {
        // Update sim with motor voltage
        climberSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
        climberSim.setInputVoltage(climberSimState.getMotorVoltage());
        climberSim.update(0.02); // 20 ms timestep

        // Sync sim state back to TalonFX sensor
        double rotorPosition = climberSim.getAngleRads() / (2 * Math.PI) * ClimberConstants.kGearRatio;
        double rotorVelocity = climberSim.getVelocityRadPerSec() / (2 * Math.PI) * ClimberConstants.kGearRatio;

        climberSimState.setRawRotorPosition(rotorPosition);
        climberSimState.setRotorVelocity(rotorVelocity);

        // Update visualization
        climberVisual.setLength(Units.metersToInches(climberSim.getAngleRads()) * 0.5); // scaled
        SmartDashboard.putData("Climber Mech", mech2d);
        SmartDashboard.putNumber("Climber Position (in)", getPosition().in(Inches));

        Logger.recordOutput("FinalComponentPoses/Climber Position", new Pose3d(0,0,getPosition().in(Meters), new Rotation3d()));
    }

    /** Percent output control (-1 to 1) */
    @Override
    public void set(double speed) {
        climberMotor.set(speed);
    }

    /** Set climber target in Distance units */
    @Override
    public void setPosition(double rots) {
        climberMotor.setControl(new PositionVoltage(rots).withSlot(0));
        SmartDashboard.putNumber("Requested Climber Position (rots)", rots);
    }

    /** Direct voltage control */
    @Override
    public void setVoltage(double volts) {
        climberMotor.setControl(new VoltageOut(volts));
    }

    /** Reset encoder position (inches) */
    @Override
    public void setEncoderPosition(double rotations) {
        climberSim.setState(Units.inchesToMeters(rotations*ClimberConstants.kInchesPerMotorRotation), 0);
    }

    /** Current climber position */
    @Override
    public Distance getPosition() {
        return Inches.of(Units.metersToInches(climberSim.getAngleRads()));
    }

    /** True if within threshold of target */
    @Override
    public boolean atTarget(double threshold) {
        return Math.abs(getPosition().in(Inches) - targetPosition.in(Inches)) < threshold;
    }

    @Override
    public SubsystemBase returnSubsystem() {
        return this;
    }

    @Override
    public double getStatorCurrent() {
        return climberMotor.getStatorCurrent().getValueAsDouble();
    }

    /** Stop the motor */
    public void stop() {
        climberMotor.setControl(new NeutralOut());
    }
}
