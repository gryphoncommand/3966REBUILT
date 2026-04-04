// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.io.File;
import java.util.List;
import java.util.Optional;
import java.util.Set;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonUtils;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Vision;
import frc.GryphonLib.ChassisAccelerations;
import frc.GryphonLib.MovementCalculations;
import frc.GryphonLib.PositionCalculations;
import frc.littletonUtils.PoseEstimator;
import frc.robot.Constants.AlignmentConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.simulation.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SimDriveSubsystem extends SubsystemBase implements DriveIO {
  // YAGSL swerve drivebase
  private final SwerveDrive swerveDrive;
  private SwerveModuleState[] desiredStates;
  private double gyroOffset = 0.0;

  private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
  private final PoseEstimator poseEstimator;
  private final Field2d field2d = new Field2d();
  private final StructArrayPublisher<SwerveModuleState> publisher;
  private final StructArrayPublisher<SwerveModuleState> desiredPublisher;
  private double currentTimestamp = Timer.getTimestamp();
  private boolean aligned = false;
  private ChassisAccelerations currentAccelerationFieldRelative = new ChassisAccelerations();
  private ChassisSpeeds prevSpeeds = new ChassisSpeeds();



  /** Creates a new DriveSubsystem. */
  public SimDriveSubsystem() {
    new Vision(this::getRealPoseSim);
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.INFO;
    try {
      swerveDrive =
          new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve"))
              .createSwerveDrive(DriveConstants.kMaxSpeedMetersPerSecond);
    } catch (Exception e) {
      DriverStation.reportError("Failed to load YAGSL swerve config.", e.getStackTrace());
      throw new RuntimeException(e);
    }

    swerveDrive.getMapleSimDrive().get().config.withBumperSize(Meters.of(DriveConstants.kTrackWidth + Units.inchesToMeters(2.5)), Meters.of(DriveConstants.kWheelBase + Units.inchesToMeters(2.5)));
    swerveDrive.setChassisDiscretization(false, false, 0.02);
    swerveDrive.setAngularVelocityCompensation(false, false, 0.0);
    swerveDrive.setHeadingCorrection(true);
    desiredStates = swerveDrive.getStates();
    swerveDrive.getMapleSimDrive().get().setSimulationWorldPose(new Pose2d(2, 2, new Rotation2d()));

    publisher = NetworkTableInstance.getDefault()
      .getStructArrayTopic("/SwerveStates", SwerveModuleState.struct).publish();
    desiredPublisher = NetworkTableInstance.getDefault()
      .getStructArrayTopic("/DesiredSwerveStates", SwerveModuleState.struct).publish();

    poseEstimator = new PoseEstimator(stateStdDevs);

    setCurrentPose(new Pose2d(2, 2, new Rotation2d()));

    Logger.recordOutput("Robot Pose", getCurrentPose());
    Logger.recordOutput("Goal Pose", field2d.getObject("Goal Pose").getPose());
    Logger.recordOutput("Current Trajectory", field2d.getObject("Current Trajectory").getPose());

    SmartDashboard.putData("Swerve Drive", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
          builder.setSmartDashboardType("SwerveDrive");

          builder.addDoubleProperty("Front Left Angle", ()->getStates()[0].angle.getDegrees(), null);
          builder.addDoubleProperty("Front Left Velocity", ()->getStates()[0].speedMetersPerSecond, null);

          builder.addDoubleProperty("Front Right Angle", ()->getStates()[1].angle.getDegrees(), null);
          builder.addDoubleProperty("Front Right Velocity", ()->getStates()[1].speedMetersPerSecond, null);

          builder.addDoubleProperty("Back Left Angle", ()->getStates()[2].angle.getDegrees(), null);
          builder.addDoubleProperty("Back Left Velocity", ()->getStates()[2].speedMetersPerSecond, null);

          builder.addDoubleProperty("Back Right Angle", ()->getStates()[3].angle.getDegrees(), null);
          builder.addDoubleProperty("Back Right Velocity", ()->getStates()[3].speedMetersPerSecond, null);

          builder.addDoubleProperty("Robot Angle", ()->getRotation().getRadians(), null);
      }
    });

    SmartDashboard.putData("Desired Swerve Positions", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
          builder.setSmartDashboardType("SwerveDrive");

          builder.addDoubleProperty("Front Left Angle", ()->getDesiredStates()[0].angle.getDegrees(), null);
          builder.addDoubleProperty("Front Left Velocity", ()->getDesiredStates()[0].speedMetersPerSecond, null);

          builder.addDoubleProperty("Front Right Angle", ()->getDesiredStates()[1].angle.getDegrees(), null);
          builder.addDoubleProperty("Front Right Velocity", ()->getDesiredStates()[1].speedMetersPerSecond, null);

          builder.addDoubleProperty("Back Left Angle", ()->getDesiredStates()[2].angle.getDegrees(), null);
          builder.addDoubleProperty("Back Left Velocity", ()->getDesiredStates()[2].speedMetersPerSecond, null);

          builder.addDoubleProperty("Back Right Angle", ()->getDesiredStates()[3].angle.getDegrees(), null);
          builder.addDoubleProperty("Back Right Velocity", ()->getDesiredStates()[3].speedMetersPerSecond, null);

          builder.addDoubleProperty("Robot Angle", ()->getRotation().getRadians(), null);
      }
    });

    field2d.getObject("Depot Passing Pose").setPose(AlignmentConstants.PassingPoseDepot);
    field2d.getObject("Outpost Passing Pose").setPose(AlignmentConstants.PassingPoseOutpost);

    RobotConfig config;
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
      return;
    }
    AutoBuilder.configure(
      this::getCurrentPose, // Robot pose supplier
      this::setCurrentPose, // Method to reset odometry (will be called if your auto has a starting pose)
      this::getCurrentSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      (speeds, feedforwards) -> driveRobotRelativeChassis(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
      new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
              new PIDConstants(3.0, 0.0, 0.0), // Translation PID constants
              new PIDConstants(3.0, 0.0, 0.0) // Rotation PID constants
      ),
      config, // The robot configuration
      () -> {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        return DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
      },
      this // Reference to this subsystem to set requirements
    );
    swerveDrive.headingCorrection = true;
    swerveDrive.getGyro().setInverted(true);
  }

  public void driveRobotRelativeChassis(ChassisSpeeds speeds) {
    swerveDrive.setChassisSpeeds(speeds);
    setDesiredStates(swerveDrive.kinematics.toSwerveModuleStates(speeds));
  }

  public AbstractDriveTrainSimulation getDriveTrainSimulation(){
    return swerveDrive.getMapleSimDrive().get();
  }

  public void driveFieldRelativeChassis(ChassisSpeeds fieldRelativeSpeeds){
    Rotation2d referenceRotation = getRotation();
    ChassisSpeeds robotRelative = ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, referenceRotation);
    swerveDrive.setChassisSpeeds(robotRelative);
    setDesiredStates(swerveDrive.kinematics.toSwerveModuleStates(robotRelative));
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    var deliveredSpeeds =
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeedDelivered, ySpeedDelivered, rotDelivered, getRotation())
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered);

    swerveDrive.drive(deliveredSpeeds, false, new Translation2d());
    setDesiredStates(swerveDrive.kinematics.toSwerveModuleStates(deliveredSpeeds));
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    SwerveModuleState[] xStates = {
      new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
      new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
      new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
      new SwerveModuleState(0, Rotation2d.fromDegrees(45))
    };
    swerveDrive.setModuleStates(xStates, false);
    setDesiredStates(xStates);
  }

  /**
   * Sets the swerve ModuleStates.
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    swerveDrive.setModuleStates(desiredStates, false);
    setDesiredStates(desiredStates);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    swerveDrive.resetDriveEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    gyroOffset = Units.radiansToDegrees(swerveDrive.imuReadingCache.getValue().getZ()); // Set current yaw as zero
    Pose2d currentPose = poseEstimator.getLatestPose();
    poseEstimator.resetPose(new Pose2d(currentPose.getX(), currentPose.getY(), DriverStation.getAlliance().get() == Alliance.Blue ? new Rotation2d() : new Rotation2d(Math.PI)));
  }

  public void setHeading(double angle) {
    gyroOffset = (angle - getRawYawDegrees()); 
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return -getRawYawDegrees() + gyroOffset;
  }

  /**
   * Returns the current chassis speeds of the robot
   * @return the robot's current speeds
   */
  public ChassisSpeeds getCurrentSpeeds(){
    return swerveDrive.getRobotVelocity();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return swerveDrive.getGyro().getYawAngularVelocity().in(DegreesPerSecond) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public SwerveModulePosition[] getPositions(){
    return swerveDrive.getModulePositions();
  }


  public SwerveModuleState[] getStates(){
    return swerveDrive.getStates();
  }

  public SwerveModuleState[] getDesiredStates(){
    SwerveModuleState[] copy = new SwerveModuleState[desiredStates.length];
    for (int i = 0; i < desiredStates.length; i++) {
      copy[i] = new SwerveModuleState(desiredStates[i].speedMetersPerSecond, desiredStates[i].angle);
    }
    return copy;
  }

  public Rotation2d getRotation(){
    // return getCurrentPose().getRotation();
    return new Rotation2d(swerveDrive.imuReadingCache.getValue().getZ()).plus(Rotation2d.fromDegrees(gyroOffset));
  }

  public void stop(){
    driveRobotRelativeChassis(new ChassisSpeeds());
  }

  public PathPlannerPath getPathFromWaypoint(Pose2d waypoint) {
    return createPath(waypoint, AutoConstants.constraints, new GoalEndState(0.0, waypoint.getRotation()));
  }

  public ChassisSpeeds getCurrentSpeedsFieldRelative(){
    boolean flipped = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
    return ChassisSpeeds.fromRobotRelativeSpeeds(getCurrentSpeeds().times(flipped ? -1 : 1), getRotation());
  }

  public Command goToPose(Pose2d goalPose){
    ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(getCurrentSpeeds(), getRotation());
    if (MovementCalculations.getVelocityMagnitude(getCurrentSpeeds()).in(MetersPerSecond) > 0.5){
      SmartDashboard.putBoolean("Included Previous Speed in Path", true);
      SmartDashboard.putNumber("Speed at start of path", MovementCalculations.getVelocityMagnitude(fieldRelativeSpeeds).in(MetersPerSecond));
      return Commands.defer(
        () -> AutoBuilder.followPath(getPathFromWaypoint(goalPose)),
        Set.of(this)
      );
    } else {
      SmartDashboard.putBoolean("Included Previous Speed in Path", false);
      SmartDashboard.putNumber("Speed at start of path", MovementCalculations.getVelocityMagnitude(fieldRelativeSpeeds).in(MetersPerSecond));
      return PathToPose(goalPose, 0.0);
    }
    
  }

  public PathPlannerPath createPath(Pose2d goalPose, PathConstraints constraints, GoalEndState endState){
    field2d.getObject("Goal Pose").setPose(goalPose);
    List<Pose2d> waypoints = List.of(getCurrentPose(), goalPose);
    field2d.getObject("Current Trajectory").setPoses(waypoints);
    

    return new PathPlannerPath(
      PathPlannerPath.waypointsFromPoses(waypoints),
      constraints,
      null,
      new GoalEndState(0.0, goalPose.getRotation())
    );
  }

  public Command PathToPose(Pose2d goalPose, double endSpeed){
    field2d.getObject("Goal Pose").setPose(goalPose);
    List<Pose2d> waypoints = List.of();
    waypoints = List.of(getCurrentPose(), goalPose);
    field2d.getObject("Current Trajectory").setPoses(waypoints);

    Command pathfindingCommand = AutoBuilder.pathfindToPose(
        goalPose,
        AutoConstants.constraints,
        endSpeed // Goal end velocity in meters/sec
    );


    return pathfindingCommand;
  }

  public Command AlignToTagFar(int goalTag){
    Pose2d goalPose;
    if (goalTag == 0){
      goalPose = getCurrentPose();
    } else {
      goalPose = PositionCalculations.getStraightOutPose(goalTag, 1.5);
    }
    return PathToPose(goalPose, 0.0);
  }

  @Override
  public void periodic() {
    field2d.getObject("Shooter Pose").setPose(getCurrentPose().transformBy(ShooterConstants.kRobotToShooter));
    SmartDashboard.putNumber("Distance To Hub (m)", PhotonUtils.getDistanceToPose(getCurrentPose(), AlignmentConstants.HubPose));
    Logger.recordOutput("Drive/Chassis Speeds", getCurrentSpeeds());
    Logger.recordOutput("Drive/Desired Chassis Speeds", DriveConstants.kDriveKinematics.toChassisSpeeds(getDesiredStates()));
    if (Vision.getResult1() != null){
      Optional<EstimatedRobotPose> visionBotPose1 = Vision.getEstimatedGlobalPoseCam1(Vision.getResult1(), getCurrentPose(), true);
      if (visionBotPose1.isPresent()){
        poseEstimator.addVisionData(List.of(visionBotPose1.get()), Vision.updateEstimationStdDevs(visionBotPose1, visionBotPose1.get().targetsUsed));
        Logger.recordOutput("PoseEst/Camera1 Pose Guess", visionBotPose1.get().estimatedPose);
      }
    } if (Vision.getResult2() != null){
      Optional<EstimatedRobotPose> visionBotPose2 = Vision.getEstimatedGlobalPoseCam2(getCurrentPose(), Vision.getResult2());
      if (visionBotPose2.isPresent()){
        poseEstimator.addVisionData(List.of(visionBotPose2.get()), Vision.updateEstimationStdDevs(visionBotPose2, visionBotPose2.get().targetsUsed));
        Logger.recordOutput("PoseEst/Camera2 Pose Guess", visionBotPose2.get().estimatedPose);
      }
    } if (Vision.getResult3() != null){
      Optional<EstimatedRobotPose> visionBotPose3 = Vision.getEstimatedGlobalPoseCam3(getCurrentPose(), Vision.getResult3());
      if (visionBotPose3.isPresent()){
        poseEstimator.addVisionData(List.of(visionBotPose3.get()), Vision.updateEstimationStdDevs(visionBotPose3, visionBotPose3.get().targetsUsed));
        field2d.getObject("Camera1 Pose Guess").setPose(visionBotPose3.get().estimatedPose.toPose2d());
      }
    } 
    
    // Update pose estimator with drivetrain sensors
    if (DriverStation.isDisabled()){
      setX();
    }
    poseEstimator.addDriveData(
      Timer.getTimestamp(),
      getCurrentSpeeds().toTwist2d(Timer.getTimestamp() - currentTimestamp)
    );

    field2d.setRobotPose(poseEstimator.getLatestPose());
    publisher.set(getStates());
    desiredPublisher.set(getDesiredStates());
    SmartDashboard.putData("Field", field2d);
    Logger.recordOutput("Drive/Current Speed", MovementCalculations.getVelocityMagnitude(getCurrentSpeeds()).magnitude());
    Logger.recordOutput("Drive/FieldRelativeSpeeds", getCurrentSpeedsFieldRelative());
    Logger.recordOutput("Drive/Real Pose", swerveDrive.getMapleSimDrive().get().getSimulatedDriveTrainPose());

    currentAccelerationFieldRelative = new ChassisAccelerations(prevSpeeds, getCurrentSpeedsFieldRelative(), Timer.getTimestamp() - currentTimestamp);

    prevSpeeds = getCurrentSpeedsFieldRelative();
    currentTimestamp = Timer.getTimestamp();

  }

  @Override
  public void simulationPeriodic() {
      // YAGSL handles simulation updates in its odometry thread.
  }

  public Pose2d getCurrentPose() {
    return poseEstimator.getLatestPose();
  }

  public Field2d getField(){
    return field2d;
  }

  /**
   * Resets the current pose to the specified pose. This should ONLY be called
   * when the robot's position on the field is known, like at the beginning of
   * a match.
   * @param newPose new pose
   */
  public void setCurrentPose(Pose2d newPose) {
    poseEstimator.resetPose(newPose);
    swerveDrive.getMapleSimDrive().get().setSimulationWorldPose(newPose);

    Rotation2d adjustedRotation = newPose.getRotation();
    if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
      adjustedRotation = adjustedRotation.plus(new Rotation2d(Math.PI));
    }
    gyroOffset = adjustedRotation.getDegrees() + getRawYawDegrees();
  }

  public void setAlign(boolean alignedNow) {
    aligned = alignedNow;
  }

  public boolean getAligned(){
    return aligned;
  }

  public double getDistanceToPose(Pose2d pose){
    return getCurrentPose().getTranslation().getDistance(pose.getTranslation());
  }

  public ChassisAccelerations getAcceleration(){
    return currentAccelerationFieldRelative;
  }

  private double getRawYawDegrees() {
    return new Rotation2d(swerveDrive.imuReadingCache.getValue().getZ()).getDegrees();
  }

  private void setDesiredStates(SwerveModuleState[] desired) {
    desiredStates = new SwerveModuleState[desired.length];
    for (int i = 0; i < desired.length; i++) {
      desiredStates[i] = new SwerveModuleState(desired[i].speedMetersPerSecond, desired[i].angle);
    }
  }

  public Pose2d getRealPoseSim(){
    return swerveDrive.getMapleSimDrive().get().getSimulatedDriveTrainPose();
  }
}
