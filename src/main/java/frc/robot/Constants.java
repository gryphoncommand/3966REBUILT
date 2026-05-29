// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import java.util.List;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.GryphonLib.AllianceFlipUtil;
import frc.GryphonLib.ShooterState;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final boolean kTuningMode = true;
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAccelerationMetersPerSecondSquared = 40.0;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final boolean fieldOriented = true;

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(21.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(25.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2), // Front Left
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),// Front Right
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),// Rear Left
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)// Rear Right
        );

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs

    
    public static final int kFrontLeftDrivingCanId = 1;
    public static final int kRearLeftDrivingCanId = 7;
    public static final int kFrontRightDrivingCanId = 3;
    public static final int kRearRightDrivingCanId = 5;

    public static final int kFrontLeftTurningCanId = 2;
    public static final int kRearLeftTurningCanId = 8;
    public static final int kFrontRightTurningCanId = 4;
    public static final int kRearRightTurningCanId = 6;
    

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = KrakenMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(3);
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 20 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = ((45.0 * 20) / (kDrivingMotorPinionTeeth * 15));
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
    public static final double TURNING_GEAR_RATIO = 46.42;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;

    public static final double kDriveDeadband = 0.15;
    public static final double kTurnDeadband = 0.07;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

        public static final PathConstraints constraints = new PathConstraints(
          3, 3,
          Units.degreesToRadians(360), Units.degreesToRadians(180));
  }

  public static final class KrakenMotorConstants {
    public static final double kFreeSpeedRpm = 6000;
  }

  public static class VisionConstants {
    public static final String kCameraName1 = "ShooterLL";
    public static final String kCameraName2 = "FrontArducam";
    public static final String kCameraName3 = "ArduL";
    // Cam mounted facing forward, half a meter forward of center, half a meter up from center,
    // pitched upward.
    private static final double camPitch1 = -Units.degreesToRadians(20);
    private static final double camYaw1 = Units.degreesToRadians(90);
    
    public static final Transform3d kRobotToCam1 =
            new Transform3d(new Translation3d(Units.inchesToMeters(9.75), Units.inchesToMeters(9.75), Units.inchesToMeters(16.732283)), new Rotation3d(0, camPitch1, camYaw1));
    public static final Transform3d kCamToRobot1 = kRobotToCam1.inverse();

    // some of these probably need to be flipped
    private static final double camPitch2 = -Units.degreesToRadians(8);
    private static final double camYaw2 = Units.degreesToRadians(-90);
    public static final Transform3d kRobotToCam2 =
            new Transform3d(new Translation3d(Units.inchesToMeters(8.25), Units.inchesToMeters(10.25), Units.inchesToMeters(19.5)), new Rotation3d(0, camPitch2, camYaw2));
    public static final Transform3d kCamToRobot2 = kRobotToCam2.inverse();

    // some of these probably need to be flipped
    private static final double camPitch3 = Units.degreesToRadians(0);
    private static final double camYaw3 = Units.degreesToRadians(15);
    public static final Transform3d kRobotToCam3 =
            new Transform3d(new Translation3d(Units.inchesToMeters(6.25), -Units.inchesToMeters(12), Units.inchesToMeters(11.75)), new Rotation3d(Math.PI, camPitch3, camYaw3));
    public static final Transform3d kCamToRobot3 = kRobotToCam3.inverse();

    public static AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    

    // The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(0.2, 0.2, 0.2);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.05, 0.05, 0.1);
  }

  public static class ShooterConstants {
    public static int kFlywheelCanID = 9;
    public static int kFollowerWheelCanID = 10;
    public static int kHoodCANID = 11;

    public static Distance kHubHeight = Inches.of(72);

    //TODO: test
    public static boolean accountForAccel = false;

    public static double kFlywheelRPMOffset = 250;
    public static double kShootDelay = 0.05;
    public static double kPhaseDelay = 0.02;


    public static double kHoodGearRatio = 100.0;
    public static double kHoodLengthMeters = Units.inchesToMeters(5);

    public static double kHoodMaxAngleDeg = 52.5;
    public static double kHoodMinAngleDeg = 22.6;
    public static double kHoodMOI = SingleJointedArmSim.estimateMOI(kHoodLengthMeters, Units.lbsToKilograms(1.5));
    public static double kDefaultFlywheelSpeed = 0.0;
    public static Transform2d kRobotToShooter = new Transform2d(0.260, 0.0, new Rotation2d());

    public static ShooterState kShooterStowState = new ShooterState(3, kHoodMinAngleDeg, 0, 1.2);
    public static ShooterState kDefaultShooterState = new ShooterState(3, 35, 1500, 1.2);
    public static ShooterState kCornerShotState = new ShooterState(5.1, 41, 3225, 1.2);
    public static ShooterState kTowerShotState = new ShooterState(3.415, 36, 2750, 1.2);
    public static ShooterState kTrenchShotState = new ShooterState(3.168, 35.0, 1900, 1.10);


    public static List<ShooterState> RealShootingValuesLow = List.of(
      new ShooterState(1.500000, 22.60, 1548.9, 0.58),
      new ShooterState(1.750000, 22.98, 1591.2, 0.67),
      new ShooterState(2.000000, 22.60, 1652.6, 0.77),
      new ShooterState(2.250000, 23.36, 1713.0, 0.82),
      new ShooterState(2.500000, 22.60, 1793.8, 0.92),
      new ShooterState(2.750000, 22.98, 1859.1, 0.97),
      new ShooterState(3.000000, 22.98, 1927.5, 1.03),
      new ShooterState(3.250000, 22.98, 1999.5, 1.09),
      new ShooterState(3.500000, 22.60, 2080.1, 1.16),
      new ShooterState(3.750000, 22.60, 2151.3, 1.22),
      new ShooterState(4.000000, 23.36, 2198.0, 1.24),
      new ShooterState(4.250000, 22.60, 2289.0, 1.32),
      new ShooterState(4.500000, 23.36, 2330.6, 1.34),
      new ShooterState(4.750000, 22.60, 2427.4, 1.42),
      new ShooterState(5.000000, 24.87, 2409.5, 1.38),
      new ShooterState(5.250000, 22.60, 2557.7, 1.51),
      new ShooterState(5.500000, 23.36, 2593.6, 1.52),
      new ShooterState(5.750000, 22.98, 2671.6, 1.58),
      new ShooterState(6.000000, 22.60, 2752.5, 1.64)
    );

    public static List<ShooterState> RealPassingValues = List.of(
      new ShooterState(1.000000, 52.50, 551.6, 0.46),
      new ShooterState(1.250000, 52.50, 678.8, 0.50),
      new ShooterState(1.500000, 52.50, 789.9, 0.54),
      new ShooterState(1.750000, 52.50, 895.4, 0.58),
      new ShooterState(2.000000, 52.50, 987.8, 0.61),
      new ShooterState(2.250000, 52.12, 1079.6, 0.65),
      new ShooterState(2.500000, 51.74, 1161.4, 0.68),
      new ShooterState(2.750000, 51.74, 1240.7, 0.71),
      new ShooterState(3.000000, 52.50, 1317.9, 0.73),
      new ShooterState(3.250000, 52.50, 1386.9, 0.76),
      new ShooterState(3.500000, 52.50, 1460.1, 0.79),
      new ShooterState(3.750000, 50.99, 1523.9, 0.83),
      new ShooterState(4.000000, 49.09, 1582.6, 0.89),
      new ShooterState(4.250000, 50.61, 1646.6, 0.89),
      new ShooterState(4.500000, 50.61, 1707.1, 0.92),
      new ShooterState(4.750000, 52.50, 1775.5, 0.91),
      new ShooterState(5.000000, 52.12, 1830.6, 0.94),
      new ShooterState(5.250000, 52.50, 1893.3, 0.96),
      new ShooterState(5.500000, 52.12, 1944.3, 0.99),
      new ShooterState(5.750000, 52.50, 2003.8, 1.00),
      new ShooterState(6.000000, 52.50, 2060.4, 1.02),
      new ShooterState(6.250000, 52.12, 2107.9, 1.05),
      new ShooterState(6.500000, 52.12, 2161.3, 1.07),
      new ShooterState(6.750000, 52.50, 2213.4, 1.08),
      new ShooterState(7.000000, 52.50, 2265.3, 1.10),
      new ShooterState(7.250000, 52.50, 2315.4, 1.12),
      new ShooterState(7.500000, 52.12, 2366.1, 1.15),
      new ShooterState(7.750000, 52.12, 2414.5, 1.17),
      new ShooterState(8.000000, 52.12, 2462.3, 1.19),
      new ShooterState(8.250000, 52.50, 2516.2, 1.20),
      new ShooterState(8.500000, 52.50, 2561.4, 1.22),
      new ShooterState(8.750000, 52.12, 2608.9, 1.24),
      new ShooterState(9.000000, 52.50, 2659.2, 1.25),
      new ShooterState(9.250000, 52.50, 2702.0, 1.27),
      new ShooterState(9.500000, 52.12, 2748.1, 1.30),
      new ShooterState(9.750000, 52.50, 2796.5, 1.31),
      new ShooterState(10.000000, 52.12, 2841.0, 1.33)
    );
  }

  public static class TurretConstants {
    public static int kTurretCanID = 18;

    // Positive is clockwise when viewed from above the robot.
    public static double kTurretGearRatio = 20.0; // TODO: Update with measured gear ratio
    public static double kTurretMinAngleDeg = -180.0;
    public static double kTurretMaxAngleDeg = 180.0;
    public static double kTurretHomeAngleDeg = 0.0;

    public static double kTurretLengthMeters = Units.inchesToMeters(8.0);
    public static double kTurretMOI = 1e-2;
    public static double kTurretHeightMeters = Units.inchesToMeters(18.0);

    // Closed-loop tuning (starting points)
    public static double kTurretP = 4.0;
    public static double kTurretI = 0.0;
    public static double kTurretD = 2.0;
    public static double kTurretMaxVolts = 12.0;
    public static double kTurretClosedLoopRampSec = 1;

    // Feedforward (rotor units). Tune with SysId for best results.
    public static double kTurretKS = 0.0;
    public static double kTurretKV = 0.0 / (KrakenMotorConstants.kFreeSpeedRpm / 60.0);
    public static double kTurretKA = 0.0;

    // Motion Magic constraints (mechanism-space)
    public static double kTurretMaxOutputRps = 2;
    public static double kTurretMaxOutputDegPerSec = Units.rotationsToDegrees(kTurretMaxOutputRps);
    public static double kTurretCruiseVelocityDegPerSec = kTurretMaxOutputDegPerSec * 0.9;
    public static double kTurretAccelerationTimeSec = 0.3;
    public static double kTurretAccelerationDegPerSec2 =
        kTurretCruiseVelocityDegPerSec / kTurretAccelerationTimeSec;

    // Motion Magic constraints converted to rotor units
    public static double kTurretCruiseVelocityRps =
        Units.degreesToRotations(kTurretCruiseVelocityDegPerSec) * kTurretGearRatio;
      public static double kTurretAccelerationRps2 =
          Units.degreesToRotations(kTurretAccelerationDegPerSec2) * kTurretGearRatio;

      // Hard-stop wrap behavior
      public static double kTurretWrapTriggerDeg = 170.0; // target near +/-180 to trigger wrap
      public static double kTurretWrapNearLimitDeg = 6.0; // turret near hard stop to allow wrap
      public static double kTurretWrapExitDeg = 3.0; // release wrap once reached limit
    }

  public static class IndexerConstants {
    public static int kSpindexerCanID = 13;
    public static int kPreIndexerCanID = 15;
    public static int kKickerCanID = 16;

    public static double kSpindexerGearRatio = 4;
    public static double kKickerGearRatio = 28/24;

    public static double kPreIndexerSpeed = 1000;
    public static double kSpindexerSpeed = 5000;
    public static double kKickerSpeed = 6500;
    

    public static double kActiveCurrentSpindexer = 20;
    
  }

  public static class ClimberConstants{
    public static double kGearRatio = 25;
    public static double kMotorRotationsPer10Inch = 70;
    public static double kInchesPerMotorRotation = 10/kMotorRotationsPer10Inch;
    public static double kMotorRotationsPerInch = 1/kInchesPerMotorRotation;
    public static double kFullUpPosition = 70;
    public static int kClimberCanID = 17;

    public static Pose2d kRightClimbPose = new Pose2d(1.2, 3.011, new Rotation2d());
    public static Pose2d kLeftClimbPose = new Pose2d(0.7, 4.410, new Rotation2d(Math.PI));
    public static Pose2d kMidClimbPose = new Pose2d(0.768, 3.781, new Rotation2d(Math.PI));

    public static Pose2d kRightPreClimb = new Pose2d(1.2, 2.191, new Rotation2d());
    public static Pose2d kLeftPreClimb = new Pose2d(0.73, 5.0, new Rotation2d(Math.PI));
  }

  public static class IntakeConstants {
    public static int kRollerCanID = 12;
    public static double kIntakeSpeedRPM = 350;


    public static double kIntakeDeployGearRatio = 560/117; // 20 * (32/50) * (14/36)
    public static double kShaftToIntakeDeployRatio = 36/16;
    public static double kIntakeDeployAngle = 0.02;
    public static double kIntakeStowAngle = 0.7;
    public static double kIntakeAgitateAngle = 0.20;
    public static int kDeployCanID = 14;
    public static double kIntakeLengthMeters = Units.inchesToMeters(14.678);
  }

  public static class AlignmentConstants {
    public static final PIDController turnPID = new PIDController(2.0, 0.00, 0.00);
    static {turnPID.enableContinuousInput(-Math.PI, Math.PI);}

    public static final Pose2d RedHubPose = new Pose2d(11.916, 4.055, new Rotation2d());
    public static final Pose2d BlueHubPose = new Pose2d(4.624, 4.055, new Rotation2d());

    public static Pose2d HubPose = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red ? RedHubPose : BlueHubPose;

    public static final Pose2d BlueAllianceZoneEnd = new Pose2d(4.3, 0, new Rotation2d());
    public static final Pose2d RedAllianceZoneEnd = new Pose2d(12.2, 0, new Rotation2d());

    public static Pose2d PassingPoseOutpost = AllianceFlipUtil.apply(new Pose2d(2.412, 2.288, new Rotation2d()));
    public static Pose2d PassingPoseDepot = AllianceFlipUtil.apply(new Pose2d(2.412, 5.607, new Rotation2d()));

    public static final double kMidFieldY = Units.feetToMeters(13.15);
    public static final double kMidFieldHubBlockWidth = 1.0;
    
    

    // Tolerances
    public static final double ANGLE_TOLERANCE_RAD = Units.degreesToRadians(5.0);
    public static final double SOTM_ANGLE_TOLERANCE_RAD = Units.degreesToRadians(10.0);
    public static final double ANG_VEL_TOLERANCE_RAD_PER_SEC = Math.toRadians(5.0);

    public static final double SPEED_VEL_TOLERANCE = DriveConstants.kMaxSpeedMetersPerSecond/6;

    public static final double MAX_DIST = 5; // Meters
    public static final double SPIN_DIST = 7;
  }

  public static class BlinkinConstants{
    public static final double blue = 0.92;
    public static final double green = 0.73;
  }
}
