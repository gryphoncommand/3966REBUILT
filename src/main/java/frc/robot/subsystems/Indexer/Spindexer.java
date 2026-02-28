package frc.robot.subsystems.Indexer;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.SpindexerConfig;
import frc.robot.Constants.IndexerConstants;

public class Spindexer extends SubsystemBase {
    private SparkFlex spindexerMotor = new SparkFlex(IndexerConstants.kSpindexerCanID, MotorType.kBrushless);
    private SparkClosedLoopController pid = spindexerMotor.getClosedLoopController();
    private RelativeEncoder encoder = spindexerMotor.getEncoder();
    private double targetReference = 0.0;
    private int simBalls = 8;

    public Spindexer(){
        spindexerMotor.configure(SpindexerConfig.SpindexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Spindexer Current", getStatorCurrent());
        SmartDashboard.putNumber("Spindexer Velocity (RPM)", getVelocity());
        SmartDashboard.putNumber("Desired Spindexer Speed", targetReference);
    }

    public void set(double speed) {
        spindexerMotor.set(speed);
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
        spindexerMotor.setVoltage(volts);
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
        return spindexerMotor.get() * RobotController.getBatteryVoltage();
    }

    public double getStatorCurrent(){
        return spindexerMotor.getOutputCurrent();
    }


    public void addBall(){
        simBalls += 1;
        if (simBalls >= 51){
            simBalls = 50;
        }
    }

    public void removeBall(){
        simBalls -= 1;
        if (simBalls < 0){
            simBalls = 0;
        }
    }

    public boolean hasBalls(){
        return simBalls > 0;
    }

    public boolean isFull(){
      return simBalls >= 50;
    }
}
