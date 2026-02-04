package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
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

import frc.robot.Configs;
import frc.robot.Constants.IntakeConstants;

public class IntakeDeploySimTalonFX extends SubsystemBase implements IntakeDeployIO {

    private final TalonFX intakeMotor =
        new TalonFX(IntakeConstants.kDeployCanID);
    private final TalonFXSimState intakeSimState =
        intakeMotor.getSimState();

    double jKgMS = 0.069; // From CAD

    private final SingleJointedArmSim intakeSim =
        new SingleJointedArmSim(
            DCMotor.getNeoVortex(1),
            IntakeConstants.kIntakeDeployGearRatio,
            jKgMS,
            IntakeConstants.kIntakeLengthMeters,
            Units.degreesToRadians(20),
            Units.degreesToRadians(190),
            false,
            Units.degreesToRadians(40));

    private double targetAngleDeg = 0;

    private final Mechanism2d mech2d = new Mechanism2d(0.6, 0.6);
    private final MechanismRoot2d root =
        mech2d.getRoot("IntakeDeployRoot", 0, 0.3);
    private final MechanismLigament2d intakeVisual =
        root.append(new MechanismLigament2d("IntakeDeploy", 0.25, 0, 8, new Color8Bit(Color.kPurple)));

    public IntakeDeploySimTalonFX() {
        intakeMotor.getConfigurator().apply(Configs.IntakeDeployConfig.deploySimConfig);
        double rotorPos =
            intakeSim.getAngleRads() / (2 * Math.PI)
            * IntakeConstants.kIntakeDeployGearRatio;

        intakeSimState.setRawRotorPosition(rotorPos);
    }

    @Override
    public void periodic() {
        intakeSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

        // Feed TalonFX output into physics sim
        intakeSim.setInputVoltage(intakeSimState.getMotorVoltage());
        intakeSim.update(0.02);

        // Sync sim position back to TalonFX sensor
        double rotorPosition =
            intakeSim.getAngleRads() / (2 * Math.PI)
            * IntakeConstants.kIntakeDeployGearRatio;

        double rotorVelocity =
            intakeSim.getVelocityRadPerSec() / (2 * Math.PI)
            * IntakeConstants.kIntakeDeployGearRatio;

        intakeSimState.setRawRotorPosition(rotorPosition);
        intakeSimState.setRotorVelocity(rotorVelocity);

        intakeVisual.setAngle(
            Units.radiansToDegrees(intakeSim.getAngleRads())
        );

        SmartDashboard.putData("Intake Deploy Mech", mech2d);
        SmartDashboard.putNumber("Intake Deploy Angle (deg)", getPosition());
        Logger.recordOutput("Intake Position", new Pose3d(-0.29, 0, 0.33, new Rotation3d(0.0, Units.degreesToRadians(getPosition()), 0.0)));
    }

    /* ---------------- IntakeDeployIO ---------------- */

    @Override
    public void set(double speed) {
        intakeMotor.set(speed);
    }

    @Override
    public void setPosition(double rotations) {
        targetAngleDeg = Units.rotationsToDegrees(rotations);
        SmartDashboard.putNumber("Requested Intake Position", targetAngleDeg);

        double simAngleDeg = 180 - targetAngleDeg;

        intakeMotor.setControl(
            new PositionVoltage(
                Units.degreesToRotations(simAngleDeg)
                * IntakeConstants.kIntakeDeployGearRatio
            ).withSlot(0)
        );
    }

    @Override
    public void setVelocity(double rpm) {
        intakeMotor.setControl(
            new com.ctre.phoenix6.controls.VelocityVoltage(rpm));
    }

    @Override
    public double getVelocity() {
        return intakeMotor.getVelocity().getValueAsDouble();
    }

    @Override
    public void setVoltage(double volts) {
        intakeMotor.setControl(new VoltageOut(volts));
    }

    @Override
    public void setEncoderPosition(double rotations) {
        double angleDeg = Units.rotationsToDegrees(rotations);
        double simAngleDeg = 180 - angleDeg;
        intakeSim.setState(
            Units.degreesToRadians(simAngleDeg), 0);
    }

    @Override
    public double getPosition() {
        return 180 - Units.radiansToDegrees(intakeSim.getAngleRads());
    }

    @Override
    public boolean atTarget(double threshold) {
        return Math.abs(getPosition() - targetAngleDeg) < threshold;
    }

    @Override
    public SubsystemBase returnSubsystem() {
        return this;
    }

    @Override
    public double getStatorCurrent() {
        return intakeMotor.getStatorCurrent().getValueAsDouble();
    }
}
