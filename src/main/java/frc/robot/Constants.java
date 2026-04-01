// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.GryphonLib.AllianceFlipUtil;
import frc.GryphonLib.ShooterInterpolator;
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
        
        public static final PathConstraints defenseConstraints = new PathConstraints(
          5, 8,
          Units.degreesToRadians(720), Units.degreesToRadians(1080));
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
    private static final double camPitch1 = -Units.degreesToRadians(15);
    private static final double camYaw1 = Units.degreesToRadians(180);
    
    public static final Transform3d kRobotToCam1 =
            new Transform3d(new Translation3d(Units.inchesToMeters(5.252), Units.inchesToMeters(9.732), Units.inchesToMeters(20.118257)), new Rotation3d(0, camPitch1, camYaw1));
    public static final Transform3d kCamToRobot1 = kRobotToCam1.inverse();

    // some of these probably need to be flipped
    private static final double camPitch2 = -Units.degreesToRadians(8);
    private static final double camYaw2 = Units.degreesToRadians(0);
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
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(0.2, 0.2, 0.5);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.05, 0.05, 0.1);
  }

  public static class ShooterConstants {
    public static int kFlywheelCanID = 9;
    public static int kFollowerWheelCanID = 10;
    public static int kThirdWheelCanID = 15;
    public static int kFourthWheelCanID = 16;

    // Needs to be fixed eventually
    public static boolean accountForAccel = false;

    // Fixed-hood, 3-wide drum shooter configuration
    public static final int kDrumMotorCount = 2;
    public static final double kFixedHoodAngleDeg = 28.0;
    public static final double kTargetFuelPerSecond = 15.0;
    public static final int kSimShooterCount = 2;
    public static final double kSimTotalBps = kTargetFuelPerSecond;
    public static final double kSimPerShooterMeanIntervalSec = (double) kSimShooterCount / kSimTotalBps;
    public static final double kSimShotIntervalJitterFrac = 0.8;
    public static final double kSimShooterXOffsetMeters = 0.37;
    public static final double kSimShooterYSpacingMeters = 0.20;
    public static final double kCoulombFrictionNm = 0.4;   // constant friction torque
    public static final double kViscousFriction = 0.00005;   // Nm per rad/s
    public static final double kGearRatio = 1.1;

    public static double kFlywheelRPMOffset = 300;
    public static double kShootDelay = 0.05;
    public static double kPhaseDelay = 0.02;

    public static double kActiveCurrentFlywheel = 60;

    public static double kDefaultFlywheelSpeed = 0.0;
    public static Transform2d kRobotToShooter = new Transform2d(0.260, 0.0, new Rotation2d(Math.PI));


    public static List<ShooterState> RealShootingValuesLow = List.of(
      new ShooterState(1.500000, 28.00, 2348.4, 0.71),
      new ShooterState(1.750000, 28.00, 2584.4, 0.66),
      new ShooterState(2.000000, 28.00, 2616.6, 0.70),
      new ShooterState(2.250000, 28.00, 2678.4, 0.75),
      new ShooterState(2.500000, 28.00, 2753.4, 0.80),
      new ShooterState(2.750000, 28.00, 2834.6, 0.86),
      new ShooterState(3.000000, 28.00, 2918.4, 0.91),
      new ShooterState(3.250000, 28.00, 3003.1, 0.96),
      new ShooterState(3.500000, 28.00, 3087.6, 1.01),
      new ShooterState(3.750000, 28.00, 3171.5, 1.05),
      new ShooterState(4.000000, 28.00, 3254.3, 1.09),
      new ShooterState(4.250000, 28.00, 3335.9, 1.13),
      new ShooterState(4.500000, 28.00, 3416.3, 1.17),
      new ShooterState(4.750000, 28.00, 3495.2, 1.21),
      new ShooterState(5.000000, 28.00, 3572.9, 1.25),
      new ShooterState(5.250000, 28.00, 3649.2, 1.28),
      new ShooterState(5.500000, 28.00, 3724.2, 1.32),
      new ShooterState(5.750000, 28.00, 3798.0, 1.35),
      new ShooterState(6.000000, 28.00, 3870.5, 1.38)
    );

    public static List<ShooterState> RealPassingValues = List.of(
      new ShooterState(1.500000, 28.00, 1538.4, 0.78),
      new ShooterState(1.750000, 28.00, 1709.7, 0.84),
      new ShooterState(2.000000, 28.00, 1866.2, 0.90),
      new ShooterState(2.250000, 28.00, 2011.1, 0.95),
      new ShooterState(2.500000, 28.00, 2146.5, 1.00),
      new ShooterState(2.750000, 28.00, 2274.2, 1.05),
      new ShooterState(3.000000, 28.00, 2395.2, 1.10),
      new ShooterState(3.250000, 28.00, 2510.5, 1.14),
      new ShooterState(3.500000, 28.00, 2620.9, 1.18),
      new ShooterState(3.750000, 28.00, 2726.8, 1.22),
      new ShooterState(4.000000, 28.00, 2828.9, 1.26),
      new ShooterState(4.250000, 28.00, 2927.4, 1.30),
      new ShooterState(4.500000, 28.00, 3022.8, 1.33),
      new ShooterState(4.750000, 28.00, 3115.2, 1.37),
      new ShooterState(5.000000, 28.00, 3205.1, 1.40),
      new ShooterState(5.250000, 28.00, 3292.5, 1.44),
      new ShooterState(5.500000, 28.00, 3377.6, 1.47),
      new ShooterState(5.750000, 28.00, 3460.7, 1.50),
      new ShooterState(6.000000, 28.00, 3541.8, 1.53),
      new ShooterState(6.250000, 28.00, 3621.2, 1.56),
      new ShooterState(6.500000, 28.00, 3698.8, 1.59),
      new ShooterState(6.750000, 28.00, 3774.8, 1.61),
      new ShooterState(7.000000, 28.00, 3849.4, 1.64),
      new ShooterState(7.250000, 28.00, 3922.5, 1.67),
      new ShooterState(7.500000, 28.00, 3994.4, 1.69),
      new ShooterState(7.750000, 28.00, 4064.9, 1.72),
      new ShooterState(8.000000, 28.00, 4134.3, 1.74),
      new ShooterState(8.250000, 28.00, 4202.5, 1.77),
      new ShooterState(8.500000, 28.00, 4269.6, 1.79),
      new ShooterState(8.750000, 28.00, 4335.7, 1.81),
      new ShooterState(9.000000, 28.00, 4400.8, 1.84),
      new ShooterState(9.250000, 28.00, 4464.9, 1.86),
      new ShooterState(9.500000, 28.00, 4528.2, 1.88),
      new ShooterState(9.750000, 28.00, 4590.5, 1.90),
      new ShooterState(10.000000, 28.00, 4652.1, 1.93)
    );


    public static ShooterState kShooterStowState = new ShooterState(3, kFixedHoodAngleDeg, 0, 1.2);
    public static ShooterState kDefaultShooterState = new ShooterState(3, kFixedHoodAngleDeg, 1800, 1.2);
    public static ShooterState kJuggleShooterState = new ShooterState(3, kFixedHoodAngleDeg, 700, 1.2);
    public static ShooterState kCornerShotState = ShooterInterpolator.interpolate(RealShootingValuesLow, 5.2);
    public static ShooterState kTowerShotState = ShooterInterpolator.interpolate(RealShootingValuesLow, 3.15);
    public static ShooterState kTrenchShotState = ShooterInterpolator.interpolate(RealShootingValuesLow, 2.3);
    public static ShooterState kBumpTowerShot = ShooterInterpolator.interpolate(RealShootingValuesLow, 3.195);
  }

  public static class IndexerConstants {
    public static int kKickerCanID = 14;
    public static int kPreIndexerCanID = 12;


    public static double kPreIndexerGearRatio = 1.0/4.0;
    public static double kKickerGearRatio = 84.0/24.0; // 84/24

    
    public static double kPreIndexerSpeed = 1000;
    public static double kKickerSpeed = 9000;
    

    // TODO: TOP PRIORITY IS TO TUNE THIS
    public static double kActiveCurrentRollerFloor = 20;
    
  }

  public static class IntakeConstants {
    public static int kRollerCanID = 11;
    public static double kIntakeSpeedRPM = 440;

    // Ts gear ratio calculations do NOT work because integer division, but it works out for now so its all good i guess Should be fixed in offseason.
    public static double kIntakeDeployGearRatio = 20*(50.0/32.0)*(36.0/16.0); // 20 then (32/50) then (16/36)
    public static double kShaftToIntakeDeployRatio = 36.0/16.0;
    public static double kIntakeDeployAngle = Units.degreesToRotations(5*kShaftToIntakeDeployRatio);
    public static double kIntakeStowAngle = Units.degreesToRotations(125*kShaftToIntakeDeployRatio);
    public static double kIntakeAgitateAngle = Units.degreesToRotations(30*kShaftToIntakeDeployRatio);

    
    public static int kDeployCanID = 13;
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
