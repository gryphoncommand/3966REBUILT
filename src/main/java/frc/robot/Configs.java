package frc.robot;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.ShooterConstants;

public final class Configs {
    public static final class MAXSwerveModule {
        public static final TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        public static final TalonFXConfiguration turnSimConfig = new TalonFXConfiguration();

        public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

        static {
                // Use module constants to calculate conversion factors and feed forward gain.
                double turningFactor = 2 * Math.PI;
                
                var slot0ConfigsDrive = driveConfig.Slot0;
                // PID + FF tuning
                slot0ConfigsDrive.kS = 0;
                slot0ConfigsDrive.kV = (12/ModuleConstants.kDrivingMotorFreeSpeedRps); // A velocity target of 1 rps results in 0.12 V output
                slot0ConfigsDrive.kA = 0;
                slot0ConfigsDrive.kP = 0;
                slot0ConfigsDrive.kI = 0; 
                slot0ConfigsDrive.kD = 0;


                driveConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 1/5;
                driveConfig.CurrentLimits.withSupplyCurrentLimit(60).withSupplyCurrentLimitEnable(true);
                
                // Motor behavior
                driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

                var slot0Configsturn = turnSimConfig.Slot0;

                turnSimConfig.ClosedLoopGeneral.ContinuousWrap = true;

                slot0Configsturn.kP = 2.0;
                slot0Configsturn.kI = 0.0;
                slot0Configsturn.kD = 0.0;

                slot0Configsturn.kS = 0.0;
                slot0Configsturn.kV = 0.0;
                slot0Configsturn.kA = 0.0;

                turnSimConfig.CurrentLimits.withSupplyCurrentLimit(20).withSupplyCurrentLimitEnable(true);
                
                // Motor behavior
                turnSimConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

                turningConfig
                        .idleMode(IdleMode.kBrake)
                        .smartCurrentLimit(20);
                turningConfig.absoluteEncoder
                        // Invert the turning encoder, since the output shaft rotates in the opposite
                        // direction of the steering motor in the MAXSwerve Module.
                        .inverted(true)
                        .positionConversionFactor(turningFactor) // radians
                        .velocityConversionFactor(turningFactor / 60.0); // radians per second
                turningConfig.closedLoop
                        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                        // These are example gains you may need to them for your own robot!
                        .pid(1, 0, 0)
                        .outputRange(-1, 1)
                        // Enable PID wrap around for the turning motor. This will allow the PID
                        // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                        // to 10 degrees will go through 0 rather than the other direction which is a
                        // longer route.
                        .positionWrappingEnabled(true)
                        .positionWrappingInputRange(0, turningFactor);
        }
    }

    public static final class SubsystemBaseConfig {
        public static final SparkMaxConfig subsystemConfig = new SparkMaxConfig();

        static {
                subsystemConfig
                        .idleMode(IdleMode.kBrake)
                        .smartCurrentLimit(80)
                        .inverted(true)
                        .openLoopRampRate(0)
                        .closedLoopRampRate(0);
                subsystemConfig.encoder
                    .positionConversionFactor(1)
                    .velocityConversionFactor(1);
                subsystemConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(0.1, 0, 0)
                    .outputRange(-0.5, 0.5);
        }
    }

    public static final class IntakeRollerConfig {
        public static final TalonFXConfiguration intakeRollerConfig = new TalonFXConfiguration();

        static {
            var slot0Configs = intakeRollerConfig.Slot0;
            // PID + FF tuning
            slot0Configs.kS = 0.0;
            slot0Configs.kV = 0.12;
            slot0Configs.kA = 0.0;
            slot0Configs.kP = 0.0;
            slot0Configs.kI = 0.0;
            slot0Configs.kD = 0.0;

            intakeRollerConfig.CurrentLimits.withSupplyCurrentLimitEnable(false);
            
            // Motor behavior
            intakeRollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
                    
        }
    }

    public static final class IntakeDeployConfig {
        public static final SparkMaxConfig IntakeDeployConfig = new SparkMaxConfig();

        static {
                IntakeDeployConfig
                        .idleMode(IdleMode.kBrake)
                        .smartCurrentLimit(40)
                        .inverted(true)
                        .openLoopRampRate(0)
                        .closedLoopRampRate(0);
                IntakeDeployConfig.encoder
                    .positionConversionFactor(IntakeConstants.kShaftToIntakeDeployRatio)
                    .velocityConversionFactor(IntakeConstants.kShaftToIntakeDeployRatio/60);
                IntakeDeployConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(1.3, 0, 0.2)
                    .outputRange(-0.4, 0.4);
        }

        public static final TalonFXConfiguration deploySimConfig = new TalonFXConfiguration();

        static {
                var slot0ConfigsDrive = deploySimConfig.Slot0;
                // PID + FF tuning
                slot0ConfigsDrive.kS = 0;
                slot0ConfigsDrive.kV = 0;
                slot0ConfigsDrive.kA = 0;
                slot0ConfigsDrive.kP = 3.0;
                slot0ConfigsDrive.kI = 0; 
                slot0ConfigsDrive.kD = 0.7;

                deploySimConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

                deploySimConfig.CurrentLimits.withSupplyCurrentLimit(50).withSupplyCurrentLimitEnable(true);
                
                // Motor behavior
                deploySimConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
                
                deploySimConfig.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
        }
    }

    public static final class FlywheelConfig {
        public static final SparkFlexConfig flywheelConfig = new SparkFlexConfig();
        public static final TalonFXConfiguration flywheelFXConfig = new TalonFXConfiguration();

        static {
                flywheelConfig
                        .idleMode(IdleMode.kCoast)
                        .smartCurrentLimit(70)
                        .inverted(false)
                        .openLoopRampRate(0)
                        .closedLoopRampRate(0);
                flywheelConfig.encoder
                    .positionConversionFactor(1)
                    .velocityConversionFactor(1);
                flywheelConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(0.0001, 0.0, 0.000001)
                    .iZone(100)
                    .outputRange(-1, 1);
                flywheelConfig.closedLoop.feedForward
                    .kS(0.01911)
                    .kV(0.00186)
                    .kA(0.5);

                var slot0ConfigsDrive = flywheelFXConfig.Slot0;
                // PID + FF tuning
                slot0ConfigsDrive.kS = 0.0;
                slot0ConfigsDrive.kV = 0.0;
                slot0ConfigsDrive.kA = 0.0;
                slot0ConfigsDrive.kP = 0.0;
                slot0ConfigsDrive.kI = 1.0;
                slot0ConfigsDrive.kD = 0.0;

                flywheelFXConfig.CurrentLimits.withSupplyCurrentLimitEnable(false);
                
                // Motor behavior
                flywheelFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        }
    }

    public static final class Climber {
        public static final TalonFXConfiguration ClimberConfig = new TalonFXConfiguration();

        static {
                var slot0Configs = ClimberConfig.Slot0;
                // PID + FF tuning
                slot0Configs.kS = 0;
                slot0Configs.kV = 0;
                slot0Configs.kA = 0;
                slot0Configs.kP = 1.0;
                slot0Configs.kI = 0;
                slot0Configs.kD = 0;

                ClimberConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

                ClimberConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
                
                // Motor behavior
                ClimberConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        }
    }

       public static final class PreIndexerConfig {
        public static final TalonFXConfiguration PreIndexerConfig = new TalonFXConfiguration();

        static {
            var slot0Configs = PreIndexerConfig.Slot0;
            // PID + FF tuning
            slot0Configs.kS = 0.0;
            slot0Configs.kV = 0.12;
            slot0Configs.kA = 0.0;
            slot0Configs.kP = 0.0;
            slot0Configs.kI = 0.0;
            slot0Configs.kD = 0.0;

            PreIndexerConfig.CurrentLimits.withSupplyCurrentLimitEnable(false);
            
            // Motor behavior
            PreIndexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
                    
        }
    }

    public static final class KickerConfig {
        public static final SparkFlexConfig kickerConfig = new SparkFlexConfig();

        static {
                kickerConfig
                        .idleMode(IdleMode.kBrake)
                        .smartCurrentLimit(40)
                        .inverted(false)
                        .openLoopRampRate(0)
                        .closedLoopRampRate(0);
                kickerConfig.encoder
                    .positionConversionFactor(IndexerConstants.kKickerGearRatio)
                    .velocityConversionFactor(IndexerConstants.kKickerGearRatio/60);
                kickerConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(0.0, 0.0, 0.0)
                    .iZone(100)
                    .outputRange(-0.9, 0.9);
                kickerConfig.closedLoop.feedForward
                    .kV(0.012)
                    .kA(0);
        }
    }

    public static final class SpindexerConfig {
        public static final SparkFlexConfig SpindexerConfig = new SparkFlexConfig();

        static {
                SpindexerConfig
                        .idleMode(IdleMode.kBrake)
                        .smartCurrentLimit(40)
                        .inverted(false)
                        .openLoopRampRate(0)
                        .closedLoopRampRate(0);
                SpindexerConfig.encoder
                    .positionConversionFactor(IndexerConstants.kSpindexerGearRatio)
                    .velocityConversionFactor(IndexerConstants.kSpindexerGearRatio/60);
                SpindexerConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(0.0003, 0.0, 0.0)
                    .iZone(100)
                    .outputRange(-0.9, 0.9);
                SpindexerConfig.closedLoop.feedForward
                    .kV(0.00186)
                    .kA(0);
        }
    }


    public static final class Hood {
        public static final TalonFXConfiguration HoodConfig = new TalonFXConfiguration();

        static {
                var slot0Configs = HoodConfig.Slot0;
                // PID + FF tuning
                slot0Configs.kS = 0.3;
                slot0Configs.kV = 0;
                slot0Configs.kA = 0;
                slot0Configs.kP = 2.5;
                slot0Configs.kI = 0.1; 
                slot0Configs.kD = 0.0;
                slot0Configs.kG = 0.3;
                slot0Configs.GravityType = GravityTypeValue.Elevator_Static;


                HoodConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

                HoodConfig.CurrentLimits.withSupplyCurrentLimit(50).withSupplyCurrentLimitEnable(true);

                HoodConfig.CurrentLimits.StatorCurrentLimit = 50.0;
                HoodConfig.CurrentLimits.StatorCurrentLimitEnable = true;

                HoodConfig.MotorOutput.PeakForwardDutyCycle = 0.13;
                HoodConfig.MotorOutput.PeakReverseDutyCycle = 0.13;

                HoodConfig.Voltage.PeakForwardVoltage = 1;
                HoodConfig.Voltage.PeakReverseVoltage = -1;
                
                // Motor behavior
                HoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

                HoodConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

                HoodConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.ShooterConstants.kHoodMaxAngleDeg * ShooterConstants.kHoodGearRatio / 360.0;
                HoodConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
                HoodConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.ShooterConstants.kHoodMinAngleDeg * ShooterConstants.kHoodGearRatio / 360.0;
                HoodConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        }
    }
}
