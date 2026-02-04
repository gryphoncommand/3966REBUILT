package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.FlywheelConfig;
import frc.robot.Constants.SpindexerConstants;

public class Spindexer extends SubsystemBase {
    private final SparkFlex spindexerMotor = new SparkFlex(SpindexerConstants.kSpindexerCanId, MotorType.kBrushless);

    private int simBalls = 8;

    public Spindexer() {
        spindexerMotor.configure(FlywheelConfig.flywheelConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void set(double speed) {
        spindexerMotor.set(speed);
    }

    public void setVelocity(double velocity) {
        spindexerMotor.getClosedLoopController().setSetpoint(velocity, ControlType.kVelocity);
    }
    
    public void setVoltage(double voltage) {
        spindexerMotor.getClosedLoopController().setSetpoint(voltage, ControlType.kVelocity);
    }

    public double getCurrent() {
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
