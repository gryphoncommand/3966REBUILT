package frc.robot.subsystems.Hood;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

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

    public class HoodSimTalonFX extends SubsystemBase implements HoodIO {

    private final TalonFX hoodMotor =
        new TalonFX(ShooterConstants.kHoodCANID);
    private final TalonFXSimState hoodSimState =
        hoodMotor.getSimState();

    double jKgMS = 0.001;// SingleJointedArmSim.estimateMOI(ShooterConstants.kHoodLengthMeters, Units.lbsToKilograms(1.5));
    

    private final SingleJointedArmSim hoodSim = new SingleJointedArmSim(DCMotor.getKrakenX44(1), ShooterConstants.kHoodGearRatio, jKgMS, ShooterConstants.kHoodLengthMeters, Units.degreesToRadians(ShooterConstants.kHoodMinAngleDeg), Units.degreesToRadians(ShooterConstants.kHoodMaxAngleDeg), false, Units.degreesToRadians(ShooterConstants.kHoodMinAngleDeg));

    private double targetAngleDeg = 0;

    private final Mechanism2d mech2d = new Mechanism2d(0.6, 0.6);
    private final MechanismRoot2d root =
        mech2d.getRoot("HoodRoot", 0.3, 0.3);
    private final MechanismLigament2d hoodVisual =
        root.append(new MechanismLigament2d("Hood", 0.25, 0, 10, new Color8Bit(Color.kBlack)));

    public HoodSimTalonFX(){
        hoodMotor.getConfigurator().apply(Configs.Hood.HoodConfig);
        double rotorPos = hoodSim.getAngleRads() / (2 * Math.PI)* ShooterConstants.kHoodGearRatio;

        hoodSimState.setRawRotorPosition(rotorPos);
    }
    
    @Override
    public void periodic() {
        hoodSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

        // Feed TalonFX output into physics sim
        hoodSim.setInputVoltage(hoodSimState.getMotorVoltage());
        hoodSim.update(0.02);

        // Sync sim position back to TalonFX sensor
        double rotorPosition =
            hoodSim.getAngleRads() / (2 * Math.PI)
            * ShooterConstants.kHoodGearRatio;

        double rotorVelocity =
            hoodSim.getVelocityRadPerSec() / (2 * Math.PI)
            * ShooterConstants.kHoodGearRatio;

        hoodSimState.setRawRotorPosition(rotorPosition);
        hoodSimState.setRotorVelocity(rotorVelocity);


        hoodVisual.setAngle(
            Units.radiansToDegrees(hoodSim.getAngleRads())
        );

        SmartDashboard.putData("Hood Mech", mech2d);
        SmartDashboard.putNumber("Hood Angle (deg)", getAngle());
    }


    @Override
    public void set(double speed) {
        hoodMotor.set(speed);
    }

    @Override
    public void setAngle(double degrees) {
        targetAngleDeg = degrees;
        SmartDashboard.putNumber("Requested Hood Position", degrees);
        hoodMotor.setControl(
            new PositionVoltage(
                Units.degreesToRotations(degrees)
                * ShooterConstants.kHoodGearRatio
            ).withSlot(0)
        );
    }

    @Override
    public void stow() {
        SmartDashboard.putNumber("Requested Hood Position", ShooterConstants.kHoodMinAngleDeg);
        hoodMotor.setControl(
            new PositionVoltage(
                Units.degreesToRotations(ShooterConstants.kHoodMinAngleDeg)
                * ShooterConstants.kHoodGearRatio
            ).withSlot(0)
        );
    }

    @Override
    public void setVoltage(double volts) {
        hoodMotor.setControl(new VoltageOut(volts));
    }

    @Override
    public void setEncoderPosition(double degrees) {
        hoodSim.setState(
            Units.degreesToRadians(degrees), 0);
    }

    @Override
    public double getAngle() {
        return Units.radiansToDegrees(hoodSim.getAngleRads());
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
        return hoodMotor.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public void stop() {
        hoodMotor.setControl(new com.ctre.phoenix6.controls.NeutralOut());
    }
}
