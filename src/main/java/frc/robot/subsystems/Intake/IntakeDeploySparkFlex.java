package frc.robot.subsystems.Intake;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;

import frc.robot.Configs;
import frc.robot.Constants.IntakeConstants;

public class IntakeDeploySparkFlex extends SubsystemBase implements IntakeDeployIO {
    public SparkFlex intakeDeployMotor = new SparkFlex(IntakeConstants.kDeployCanID, MotorType.kBrushless);
    public RelativeEncoder intakeDeployEncoder = intakeDeployMotor.getEncoder();
    public SparkClosedLoopController pid;

    public IntakeDeploySparkFlex(){
        intakeDeployMotor.configure(Configs.IntakeDeployConfig.IntakeDeployConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        pid = intakeDeployMotor.getClosedLoopController();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Position", intakeDeployEncoder.getPosition());
    }

    @Override
    public void set(double speed) {
        intakeDeployMotor.set(speed);
    }

    @Override
    public double getStatorCurrent() {
        return intakeDeployMotor.getOutputCurrent();
    }

    @Override
    public void setPosition(double position) {
        pid.setSetpoint(position, ControlType.kPosition);
    }

    @Override
    public void setVelocity(double rpm) {
        pid.setSetpoint(rpm, ControlType.kVelocity);
    }

    @Override
    public double getVelocity() {
        return intakeDeployEncoder.getVelocity();
    }

    @Override
    public double getPosition() {
        return intakeDeployEncoder.getPosition();
    }

    @Override
    public void setVoltage(double volts) {
        intakeDeployMotor.setVoltage(volts);
    }

    @Override
    public SubsystemBase returnSubsystem() {
        return this;
    }

    @Override
    public void setEncoderPosition(double rotations) {
        intakeDeployEncoder.setPosition(rotations);
    }

    @Override
    public boolean atTarget(double threshold) {
        if (pid.getControlType() == ControlType.kVelocity) {
            return Math.abs(getVelocity() - pid.getSetpoint()) < threshold;
        } else if (pid.getControlType() == ControlType.kPosition) {
            return Math.abs(getPosition() - pid.getSetpoint()) < threshold;
        } else {
            return false;
        }
    }
}
