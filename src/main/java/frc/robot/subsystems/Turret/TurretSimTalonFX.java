package frc.robot.subsystems.Turret;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
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
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TurretConstants;

public class TurretSimTalonFX extends SubsystemBase implements TurretIO {

    private final TalonFX turretMotor =
        new TalonFX(TurretConstants.kTurretCanID);
    private final TalonFXSimState turretSimState =
        turretMotor.getSimState();

    private final SingleJointedArmSim turretSim = new SingleJointedArmSim(
        DCMotor.getKrakenX44(1),
        TurretConstants.kTurretGearRatio,
        TurretConstants.kTurretMOI,
        TurretConstants.kTurretLengthMeters,
        Units.degreesToRadians(TurretConstants.kTurretMinAngleDeg),
        Units.degreesToRadians(TurretConstants.kTurretMaxAngleDeg),
        false,
        Units.degreesToRadians(TurretConstants.kTurretHomeAngleDeg));

    private double targetAngleDeg = TurretConstants.kTurretHomeAngleDeg;

    private final Mechanism2d mech2d = new Mechanism2d(0.6, 0.6);
    private final MechanismRoot2d root =
        mech2d.getRoot("TurretRoot", 0.3, 0.3);
    private final MechanismLigament2d turretVisual =
        root.append(new MechanismLigament2d("Turret", 0.25, 0, 6, new Color8Bit(Color.kPurple)));

    public TurretSimTalonFX() {
        turretMotor.getConfigurator().apply(Configs.Turret.TurretConfig.withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive)));
        double rotorPos =
            turretSim.getAngleRads() / (2 * Math.PI)
            * TurretConstants.kTurretGearRatio;

        turretSimState.setRawRotorPosition(rotorPos);
    }
    
    @Override
    public void periodic() {
        turretSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

        // Feed TalonFX output into physics sim
        turretSim.setInputVoltage(turretSimState.getMotorVoltage());
        turretSim.update(0.02);

        // Sync sim position back to TalonFX sensor
        double rotorPosition =
            turretSim.getAngleRads() / (2 * Math.PI)
            * TurretConstants.kTurretGearRatio;

        double rotorVelocity =
            turretSim.getVelocityRadPerSec() / (2 * Math.PI)
            * TurretConstants.kTurretGearRatio;

        turretSimState.setRawRotorPosition(rotorPosition);
        turretSimState.setRotorVelocity(rotorVelocity);


        turretVisual.setAngle(getAngle());

        SmartDashboard.putData("Turret Mech", mech2d);
        Logger.recordOutput("Turret/Closed Loop Error",turretMotor.getClosedLoopError().getValueAsDouble());
        Logger.recordOutput("Turret/Desired Turret Angle", targetAngleDeg);
        Logger.recordOutput("Turret/Turret Angle (deg)", getAngle());   
        Logger.recordOutput("Turret/Applied Turret Volts", turretSimState.getMotorVoltage());
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
        turretMotor.setControl(
            new PositionVoltage(
                Units.degreesToRotations(degrees)
                * TurretConstants.kTurretGearRatio
            ).withSlot(0)
        );
    }

    @Override
    public void setVoltage(double volts) {
        turretMotor.setControl(new VoltageOut(volts));
    }

    @Override
    public void setEncoderPosition(double degrees) {
        turretSim.setState(
            Units.degreesToRadians(degrees), 0);
    }

    @Override
    public double getAngle() {
        return Units.radiansToDegrees(turretSim.getAngleRads());
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
