package frc.robot.subsystems.Drive;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.GoalEndState;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.GryphonLib.ChassisAccelerations;

/**
 * DriveIO provides an abstraction over the concrete Drive subsystem
 * implementations (real and sim). Methods are a subset of the public
 * drive API used by the rest of the codebase.
 */
public interface DriveIO extends Subsystem {
  void driveRobotRelativeChassis(ChassisSpeeds speeds);

  void driveFieldRelativeChassis(ChassisSpeeds fieldRelativeSpeeds);

  void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative);

  void setX();

  void setModuleStates(SwerveModuleState[] desiredStates);

  void resetEncoders();

  void zeroHeading();

  void setHeading(double angle);

  double getHeading();

  ChassisSpeeds getCurrentSpeeds();

  double getTurnRate();

  SwerveModulePosition[] getPositions();

  SwerveModuleState[] getStates();

  SwerveModuleState[] getDesiredStates();

  Rotation2d getRotation();

  void stop();

  PathPlannerPath getPathFromWaypoint(Pose2d waypoint);

  ChassisSpeeds getCurrentSpeedsFieldRelative();

  Command goToPose(Pose2d goalPose);

  PathPlannerPath createPath(Pose2d goalPose, PathConstraints constraints, GoalEndState endState);

  Command PathToPose(Pose2d goalPose, double endSpeed);

  Command AlignToTagFar(int goalTag);

  Pose2d getCurrentPose();

  Field2d getField();

  void setCurrentPose(Pose2d newPose);

  void setAlign(boolean alignedNow);

  boolean getAligned();

  double getDistanceToPose(Pose2d pose);

  ChassisAccelerations getAcceleration();
}