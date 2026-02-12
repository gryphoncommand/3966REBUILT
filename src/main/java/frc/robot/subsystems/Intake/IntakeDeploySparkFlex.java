package frc.robot.subsystems.Intake;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.AbsoluteEncoder;
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
    public AbsoluteEncoder m_absoluteEncoder;

    private final Mechanism2d mech2d = new Mechanism2d(0.6, 0.6);
    private final MechanismRoot2d root =
        mech2d.getRoot("IntakeDeployRoot", 0, 0.3);
    private final MechanismLigament2d intakeVisual =
        root.append(new MechanismLigament2d("IntakeDeploy", 0.25, 0, 8, new Color8Bit(Color.kPurple)));

    public IntakeDeploySparkFlex(){
        intakeDeployMotor.configure(Configs.IntakeDeployConfig.IntakeDeployConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        pid = intakeDeployMotor.getClosedLoopController();

        initializeEncoderFromAbsolute();
        SmartDashboard.putData("Intake Deploy Mech", mech2d);
    }

    private void initializeEncoderFromAbsolute() {
        m_absoluteEncoder = intakeDeployMotor.getAbsoluteEncoder();
        double absRotations = -m_absoluteEncoder.getPosition();
        double absDegrees = Units.rotationsToDegrees(absRotations);
        double gearRatio = 36.0 / 14.0;
        double mechDegrees = (absDegrees * gearRatio) + 140;

        SmartDashboard.putNumber("Absolute Encoder Reported Shaft Degrees", absDegrees);

        SmartDashboard.putNumber("Absolute Encoder Reported Mechanism Degrees", mechDegrees);

        intakeDeployEncoder.setPosition(Units.degreesToRotations(mechDegrees));
    }

    @Override
    public void periodic() {
        double absRotations = -m_absoluteEncoder.getPosition();
        double absDegrees = Units.rotationsToDegrees(absRotations);
        double gearRatio = 36.0 / 14.0;
        double mechDegrees = (absDegrees * gearRatio) + 140;
        SmartDashboard.putNumber("Absolute Encoder Reported Degrees", mechDegrees);
        SmartDashboard.putNumber("Intake Position", intakeDeployEncoder.getPosition());
        SmartDashboard.putNumber("Desired Intake Position", pid.getSetpoint());
        intakeVisual.setAngle(Units.rotationsToDegrees(getPosition()));
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
