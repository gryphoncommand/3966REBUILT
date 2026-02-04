package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.IntakeConstants;

public class IntakeRollersTalonFX extends SubsystemBase {
  private final TalonFX rollerMotor = new TalonFX(IntakeConstants.kRollerCanID);

  public VelocityVoltage m_controlRequest = new VelocityVoltage(3000);

  private double targetReference = 0;
  private ControlType currentControlType = ControlType.kVelocity;

  public IntakeRollersTalonFX() {
    rollerMotor.getConfigurator().apply(Configs.IntakeRollerConfig.intakeRollerConfig);
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
    rpm = rpm * 43/18;
    
    rollerMotor.setControl(m_controlRequest.withVelocity(RPM.of(rpm * 43/18)));
  }


  public void setVoltage(double volts) {
    rollerMotor.setVoltage(volts);
  }

  public double getVelocity() {
    return rollerMotor.getVelocity().getValueAsDouble();
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

  // TODO: These should go to the spindexer
}
