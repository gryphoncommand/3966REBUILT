package frc.robot.subsystems.Indexer;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.KickerConfig;
import frc.robot.Constants.IndexerConstants;

public class Kicker extends SubsystemBase {
    private SparkFlex kickerMotor = new SparkFlex(IndexerConstants.kKickerCanID, MotorType.kBrushless);
    private SparkClosedLoopController pid = kickerMotor.getClosedLoopController();
    private RelativeEncoder encoder = kickerMotor.getEncoder();
    private double targetReference = 0.0;

    public Kicker(){
        kickerMotor.configure(KickerConfig.kickerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Indexing/Kicker Velocity (RPM)", getVelocity());
        Logger.recordOutput("Indexing/Desired Kicker Speed", targetReference);
        Logger.recordOutput("Indexing/Kicker Applied Output", kickerMotor.get());
    }

    public void set(double speed) {
        kickerMotor.set(speed);
    }

    public void setVelocity(double rpm) {
        targetReference = rpm;
        pid.setSetpoint(rpm, ControlType.kVelocity);
    }

    public void setPosition(double position){
        pid.setSetpoint(position, ControlType.kPosition);
        
        targetReference = position;
    }

    public void setVoltage(double volts) {
        kickerMotor.setVoltage(volts);
    }

    public double getPosition() {
        return encoder.getPosition();
    }

    public void setEncoderPosition(double position) {
        encoder.setPosition(position);
    }

    public double getVelocity() {
        return encoder.getVelocity();
    }

    public boolean atTarget(double threshold) {
        ControlType currentControlType = pid.getControlType();
        if (currentControlType == ControlType.kVelocity) {
            return Math.abs(getVelocity() - targetReference) < threshold;
        } else {
            return false;
        }
    }

    public SubsystemBase returnSubsystem() {
        return this;
    }

    public double getVoltage() {
        return kickerMotor.get() * RobotController.getBatteryVoltage();
    }

    public double getStatorCurrent(){
        return kickerMotor.getOutputCurrent();
    }
}
