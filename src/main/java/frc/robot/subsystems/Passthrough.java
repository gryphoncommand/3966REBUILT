package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.FlywheelConfig;
import frc.robot.Constants.PassthroughConstants;

public class Passthrough extends SubsystemBase {
    private final SparkFlex passthroughMotor = new SparkFlex(PassthroughConstants.kPassthroughCanId, MotorType.kBrushless);

    public Passthrough() {
        passthroughMotor.configure(FlywheelConfig.flywheelConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void set(double speed) {
        passthroughMotor.set(speed);
    }

    public void setVelocity(double velocity) {
        passthroughMotor.getClosedLoopController().setSetpoint(velocity, ControlType.kVelocity);
    }
    
    public void setVoltage(double voltage) {
        passthroughMotor.getClosedLoopController().setSetpoint(voltage, ControlType.kVelocity);
    }

    public double getCurrent() {
        return passthroughMotor.getOutputCurrent();
    }
}
