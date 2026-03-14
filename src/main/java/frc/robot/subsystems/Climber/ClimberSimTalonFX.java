package frc.robot.subsystems.Climber;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

import org.littletonrobotics.junction.Logger;

import frc.robot.Configs;
import frc.robot.Constants.ClimberConstants;

public class ClimberSimTalonFX extends SubsystemBase implements ClimberIO {

    private final TalonFX climberMotor =
        new TalonFX(ClimberConstants.kClimberCanID);

    private final TalonFXSimState climberSimState =
        climberMotor.getSimState();

    // 🔥 Use ElevatorSim (correct physics model)
    private final ElevatorSim climberSim =
        new ElevatorSim(
            DCMotor.getKrakenX60(1),
            ClimberConstants.kGearRatio,
            30,
            Units.inchesToMeters(1), // drum radius
            0.0,
            Units.inchesToMeters(10), // max height
            false,
            0.0
        );

    private double targetRots = 0.0;

    private final Mechanism2d mech2d = new Mechanism2d(0.4, 0.8);
    private final MechanismRoot2d root =
        mech2d.getRoot("climberRoot", 0.2, 0.05);

    private final MechanismLigament2d climberVisual =
        root.append(new MechanismLigament2d("climber", 0.0, 90));

    public ClimberSimTalonFX() {
        climberMotor.getConfigurator().apply(Configs.Climber.ClimberConfig.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive));
    }

    @Override
    public void periodic() {

        // Supply voltage
        climberSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

        var motorVoltage = climberSimState.getMotorVoltageMeasure();
        climberSim.setInputVoltage(motorVoltage.in(Volts)/2);
        climberSim.update(0.02);

        double heightMeters = climberSim.getPositionMeters();
        double inches = Units.metersToInches(heightMeters);

        double motorRotations =
            inches / ClimberConstants.kInchesPerMotorRotation;

        double motorRPS =
            Units.metersToInches(climberSim.getVelocityMetersPerSecond())
            / ClimberConstants.kInchesPerMotorRotation;

        // Apply to Talon sim
        climberSimState.setRawRotorPosition(motorRotations);
        climberSimState.setRotorVelocity(motorRPS);

        // Visual
        climberVisual.setLength(heightMeters * 2.0);

        SmartDashboard.putData("Climber Mech", mech2d);
        Logger.recordOutput("Climber/Climber Height (in)", inches);
        Logger.recordOutput("Climber/Climber Motor Rots", motorRotations);
        Logger.recordOutput("Climber/Applied Volts", motorVoltage.in(Volts)/2);

        Logger.recordOutput(
            "FinalComponentPoses/Climber Position",
            new Pose3d(0, 0, heightMeters, new Rotation3d())
        );
    }

    @Override
    public void set(double speed) {
        climberMotor.set(speed);
    }

    @Override
    public void setPosition(double rots) {
        targetRots = rots;
        SmartDashboard.putNumber("Requested Climber Position (rots)", rots);
        climberMotor.setControl(new PositionVoltage(rots));
    }

    @Override
    public void setVoltage(double volts) {
        climberMotor.setControl(new VoltageOut(volts));
    }

    @Override
    public void setEncoderPosition(double rotations) {
        climberMotor.setPosition(rotations);

        double inches = rotations * ClimberConstants.kInchesPerMotorRotation;
        climberSim.setState(Units.inchesToMeters(inches), 0);
    }

    @Override
    public double getPosition() {
        double rotations =
            climberMotor.getPosition().getValue().in(Rotations);

        return rotations;
    }

    @Override
    public boolean atTarget(double threshold) {
        return Math.abs(
            climberMotor.getPosition().getValue().in(Rotations)
            - targetRots
        ) < threshold;
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