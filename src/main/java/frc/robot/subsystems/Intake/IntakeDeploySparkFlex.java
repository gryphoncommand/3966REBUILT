package frc.robot.subsystems.Intake;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;

import frc.robot.Configs;
import frc.robot.Constants.IntakeConstants;

public class IntakeDeploySparkFlex extends SubsystemBase implements IntakeDeployIO {
    public SparkFlex intakeDeployMotor = new SparkFlex(IntakeConstants.kDeployCanID, MotorType.kBrushless);
    public SparkClosedLoopController pid;
    public AbsoluteEncoder m_absoluteEncoder;

    public IntakeDeploySparkFlex(){
        intakeDeployMotor.configure(Configs.IntakeDeployConfig.IntakeDeployConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        pid = intakeDeployMotor.getClosedLoopController();
        m_absoluteEncoder = intakeDeployMotor.getAbsoluteEncoder();
    }

    private double getMechPosition(double absRotations) {
        double absDegrees = Units.rotationsToDegrees(absRotations);
        double gearRatio = IntakeConstants.kShaftToIntakeDeployRatio;
        double mechDegrees = (absDegrees / gearRatio);

        return mechDegrees;
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Intake/Intake Deploy Angle (deg)", getPosition());
        Logger.recordOutput("Intake/Requested Intake Position", getMechPosition(pid.getSetpoint()));
        Logger.recordOutput("FinalComponentPoses/Intake Position", new Pose3d(-0.29, 0, 0.33, new Rotation3d(0.0, Units.degreesToRadians(getPosition()), 0.0)));
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
        return m_absoluteEncoder.getVelocity();
    }

    @Override
    public double getPosition() {
        return getMechPosition(m_absoluteEncoder.getPosition());
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
        // Idk :/
    }

    @Override
    public boolean atTarget(double threshold) {
        if (pid.getControlType() == ControlType.kVelocity) {
            return Math.abs(m_absoluteEncoder.getVelocity() - pid.getSetpoint()) < threshold;
        } else if (pid.getControlType() == ControlType.kPosition) {
            return Math.abs(m_absoluteEncoder.getPosition() - pid.getSetpoint()) < threshold;
        } else {
            return false;
        }
    }
}
