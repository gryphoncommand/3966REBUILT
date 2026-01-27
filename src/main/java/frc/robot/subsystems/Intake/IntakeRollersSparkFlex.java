package frc.robot.subsystems.Intake;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.IntakeConstants;

public class IntakeRollersSparkFlex extends SubsystemBase {
  private final SparkFlex rollerMotor = new SparkFlex(IntakeConstants.kRollerCanID, MotorType.kBrushless);
  private final SparkClosedLoopController closedLoopController = rollerMotor.getClosedLoopController();
  private final RelativeEncoder encoder = rollerMotor.getEncoder();

  private double targetReference = 0;
  private ControlType currentControlType = ControlType.kVelocity;

  public IntakeRollersSparkFlex() {
    rollerMotor.configure(Configs.IntakeRollerConfig.intakeRollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake Roller Desired Velocity (RPM)", targetReference);
    SmartDashboard.putNumber("Intake Roller Velocity (RPM)", getVelocity());
  }

  public void set(double speed) {
    rollerMotor.set(speed);
  }

  public void setVelocity(double rpm) {
    targetReference = rpm;
    closedLoopController.setSetpoint(rpm, ControlType.kVelocity);
    currentControlType = ControlType.kVelocity;
  }

  public void setPosition(double position){
    closedLoopController.setSetpoint(position, ControlType.kPosition);
    
    targetReference = position;
    currentControlType = ControlType.kPosition;
}

  public void setVoltage(double volts) {
    rollerMotor.setVoltage(volts);
  }

  public void setEncoderPosition(double position) {
    encoder.setPosition(position);
  }

  public double getVelocity() {
    return encoder.getVelocity();
  }

  public void intake(){
    setVelocity(IntakeConstants.kIntakeSpeedRPM);
    // set(0.2);
  }

  public void stop(){
    setVelocity(0);
    // set(0.0);
  }

  public boolean atTarget(double threshold) {
    if (currentControlType == ControlType.kVelocity) {
        return Math.abs(getVelocity() - targetReference) < threshold;
    } else {
        return false;
    }
  }

  public SubsystemBase returnSubsystem() {
    return this;
  }
}
