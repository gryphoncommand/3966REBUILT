package frc.robot;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

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

                var motionMagicConfigs = driveConfig.MotionMagic;

                motionMagicConfigs.MotionMagicAcceleration = frc.robot.subsystems.MAXSwerveModule.mpsToRps(DriveConstants.kMaxSpeedMetersPerSecond*5);

                driveConfig.CurrentLimits.withSupplyCurrentLimit(90).withSupplyCurrentLimitEnable(true);
                
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

    public static final class FlywheelConfig {
        public static final SparkFlexConfig flywheelConfig = new SparkFlexConfig();
        public static final TalonFXConfiguration flywheelFXConfig = new TalonFXConfiguration();

        static {
                flywheelConfig
                        .idleMode(IdleMode.kBrake)
                        .smartCurrentLimit(80)
                        .inverted(true)
                        .openLoopRampRate(0)
                        .closedLoopRampRate(0);
                flywheelConfig.encoder
                    .positionConversionFactor(1)
                    .velocityConversionFactor(1);
                flywheelConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // These are example gains need to them for your own robot!
                    .pid(5, 2, 0)
                    .outputRange(-0.95, 0.95);

                var slot0ConfigsDrive = flywheelFXConfig.Slot0;
                // PID + FF tuning
                slot0ConfigsDrive.kS = 0.0;
                slot0ConfigsDrive.kV = 0.0;
                slot0ConfigsDrive.kA = 0.0;
                slot0ConfigsDrive.kP = 10.0;
                slot0ConfigsDrive.kI = 0.0; 
                slot0ConfigsDrive.kD = 1.0;

                flywheelFXConfig.CurrentLimits.withSupplyCurrentLimitEnable(false);
                
                // Motor behavior
                flywheelFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        }
    }

    public static final class Hood {
        public static final TalonFXConfiguration HoodConfig = new TalonFXConfiguration();

        static {
                // Use module constants to calculate conversion factors and feed forward gain.                
                var slot0ConfigsDrive = HoodConfig.Slot0;
                // PID + FF tuning
                slot0ConfigsDrive.kS = 0;
                slot0ConfigsDrive.kV = 0;
                slot0ConfigsDrive.kA = 0;
                slot0ConfigsDrive.kP = 1.0;
                slot0ConfigsDrive.kI = 0; 
                slot0ConfigsDrive.kD = 0;

                HoodConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

                HoodConfig.CurrentLimits.withSupplyCurrentLimit(90).withSupplyCurrentLimitEnable(true);
                
                // Motor behavior
                HoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        }
    }
}
