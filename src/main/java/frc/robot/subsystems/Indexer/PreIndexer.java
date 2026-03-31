package frc.robot.subsystems.Indexer;

import static edu.wpi.first.units.Units.RPM;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.IndexerConstants;

public class PreIndexer extends SubsystemBase {

    private SparkFlex preIndexerMotor =
        new SparkFlex(IndexerConstants.kPreIndexerCanID, MotorType.kBrushless);

    private RelativeEncoder encoder = preIndexerMotor.getEncoder();
    private SparkClosedLoopController pid = preIndexerMotor.getClosedLoopController();

    private double targetRPM = 0.0;
    private int simBalls = 8;

    public PreIndexer() {
        preIndexerMotor.configure(Configs.PreIndexerConfig.preIndexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Indexing/PreIndexer RPM", encoder.getVelocity());
        Logger.recordOutput("Indexing/Desired PreIndexer RPM", targetRPM);
        Logger.recordOutput("Indexing/PreIndexer Applied Output", preIndexerMotor.getAppliedOutput());
    }

    public void set(double speed) {
        preIndexerMotor.set(speed);
    }

    public void setVelocity(double rpm) {
        targetRPM = rpm;
        pid.setSetpoint(rpm, ControlType.kVelocity);
    }

    public AngularVelocity getVelocity() {
        return RPM.of(encoder.getVelocity());
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public double getPosition(){
        return encoder.getPosition();
    }

    public void setVoltage(double volts){
        preIndexerMotor.setVoltage(volts);
    }

    public double getVoltage() {
        return preIndexerMotor.getAppliedOutput() * preIndexerMotor.getBusVoltage();
    }

    public boolean atTarget(double threshold) {
        return Math.abs(encoder.getVelocity() - targetRPM) < threshold;
    }

    public void addBall() {
        simBalls += 1;
        if (simBalls >= 41) {
            simBalls = 40;
        }
    }

    public void removeBall() {
        simBalls -= 1;
        if (simBalls < 0) {
            simBalls = 0;
        }
    }

    public double getBalls() {
        return simBalls;
    }

    public boolean isFull() {
        return simBalls >= 40;
    }
}