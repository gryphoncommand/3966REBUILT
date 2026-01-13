// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import frc.robot.Configs;
import frc.robot.Constants.ModuleConstants;

public class SimSwerveModule implements SwerveModuleIO {
  private final TalonFX m_drivingKraken;
  private final TalonFXSimState m_drivingSim;
  private final TalonFX m_turningMotor;
  private final TalonFXSimState m_turningSim;

  final VelocityVoltage m_driveRequest = new VelocityVoltage(0);
  // final VelocityVoltage m_driveRequest = new VelocityVoltage(0);
  private final PositionVoltage m_turnControlRequest = new PositionVoltage(0);

  private Rotation2d m_simulatedAzimuth = new Rotation2d();

  private final DCMotorSim m_motorSimModel = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60(1), 1e-6, ModuleConstants.kDrivingMotorReduction
        ),
        DCMotor.getKrakenX60(1)
    );

    private final DCMotorSim m_turningMotorModel = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getNeo550(1), 1e-6, ModuleConstants.TURNING_GEAR_RATIO
        ),
        DCMotor.getNeo550(1)
    );

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());


  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public SimSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    m_drivingKraken = new TalonFX(drivingCANId);
    m_drivingSim = m_drivingKraken.getSimState();
    m_drivingSim.setSupplyVoltage(Volts.of(12));

    m_turningMotor = new TalonFX(turningCANId);
    m_turningSim = m_turningMotor.getSimState();
    m_turningSim.setSupplyVoltage(Volts.of(12));

    // Apply the respective configurations to the SPARKS. Reset parameters before
    // applying the configuration to bring the SPARK to a known good state. Persist
    // the settings to the SPARK to avoid losing them on a power cycle.
    m_drivingKraken.getConfigurator().apply(Configs.MAXSwerveModule.driveConfig);
    m_turningMotor.getConfigurator().apply(Configs.MAXSwerveModule.turnSimConfig);

    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d();
    m_drivingKraken.setPosition(0);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    //double wheelSpeed = Math.copySign(m_correctedDesiredState.speedMetersPerSecond, rpsToMps(m_drivingKraken.getVelocity().getValueAsDouble()));
    double wheelSpeed = rpsToMps(m_drivingKraken.getVelocity().getValueAsDouble());
    Rotation2d wheelAngle = new Rotation2d(Math.IEEEremainder(Units.rotationsToRadians(m_turningMotor.getPosition().getValueAsDouble()) - m_chassisAngularOffset, 2*Math.PI));
    // Rotation2d wheelAngle = m_simulatedAzimuth.minus(Rotation2d.fromRadians(m_chassisAngularOffset));

    return new SwerveModuleState(wheelSpeed, wheelAngle);
  }

  public SwerveModuleState getDesiredState(){
    return new SwerveModuleState(m_desiredState.speedMetersPerSecond, m_desiredState.angle);
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        m_drivingKraken.getPosition().getValueAsDouble(),
        m_simulatedAzimuth);
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    correctedDesiredState.optimize(new Rotation2d(Units.rotationsToRadians(m_turningMotor.getPosition().getValueAsDouble())));

    // Command driving and turning motors towards their respective setpoints.
    m_driveRequest.Slot = 0;
    m_turnControlRequest.Slot = 0;
    m_drivingKraken.setControl(
      m_driveRequest.withVelocity(mpsToRps(correctedDesiredState.speedMetersPerSecond)).withEnableFOC(true)
    );
    m_turningMotor.setControl(
      m_turnControlRequest.withPosition(correctedDesiredState.angle.getRotations())
    );

    m_simulatedAzimuth = correctedDesiredState.angle;
    m_desiredState = desiredState;
  }

  static double mpsToRps(double mps) {
    return (mps / (ModuleConstants.kWheelDiameterMeters * Math.PI)) * ModuleConstants.kDrivingMotorReduction;
  }

  static double rpsToMps(double rps) {
    return rps * ((ModuleConstants.kWheelDiameterMeters * Math.PI) / ModuleConstants.kDrivingMotorReduction);
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingKraken.setPosition(0);
  }

  public void simulationPeriodic() {
    // get the motor voltage of the TalonFX
    var motorVoltageTurn = m_turningSim.getMotorVoltageMeasure();

    // use the motor voltage to calculate new position and velocity
    // using WPILib's DCMotorSim class for physics simulation
    m_turningMotorModel.setInputVoltage(motorVoltageTurn.in(Volts));
    m_turningMotorModel.update(0.02);

    // apply the new rotor position and velocity to the TalonFX;
    // note that this is rotor position/velocity (before gear ratio), but
    // DCMotorSim returns mechanism position/velocity (after gear ratio)
    double mechPos = m_turningMotorModel.getAngularPosition().in(Rotations);
    double mechVel = m_turningMotorModel.getAngularVelocity().in(RotationsPerSecond);

    m_turningSim.setRawRotorPosition(mechPos*ModuleConstants.TURNING_GEAR_RATIO);
    m_turningSim.setRotorVelocity(mechVel*ModuleConstants.TURNING_GEAR_RATIO);

    // get the motor voltage of the TalonFX
    var motorVoltage = m_drivingSim.getMotorVoltageMeasure();

    // use the motor voltage to calculate new position and velocity
    // using WPILib's DCMotorSim class for physics simulation
    m_motorSimModel.setInputVoltage(motorVoltage.in(Volts));
    m_motorSimModel.update(0.02);

    // apply the new rotor position and velocity to the TalonFX;
    // note that this is rotor position/velocity (before gear ratio), but
    // DCMotorSim returns mechanism position/velocity (after gear ratio)
    m_drivingSim.setRawRotorPosition(m_motorSimModel.getAngularPosition().in(Rotations) * ModuleConstants.kDrivingMotorReduction);
    m_drivingSim.setRotorVelocity(m_motorSimModel.getAngularVelocity().in(RotationsPerSecond) * ModuleConstants.kDrivingMotorReduction);
  }
}
