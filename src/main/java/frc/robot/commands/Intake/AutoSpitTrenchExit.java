package frc.robot.commands.Intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.FuelSim;
import frc.littletonUtils.FieldConstants;
import frc.littletonUtils.HubShiftUtil;
import frc.robot.Constants.AlignmentConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Robot;
import frc.robot.subsystems.Drive.DriveIO;
import frc.robot.subsystems.Drive.SimDriveSubsystem;
import frc.robot.subsystems.Indexer.PreIndexer;
import frc.robot.subsystems.Intake.IntakeRollersTalonFX;

/**
 * Watches for the "leaving alliance zone while aligning to trench" situation and, when safe,
 * schedules a short-lived outtake command to reverse the intake rollers + preindexer and (in sim)
 * eject fuel back onto the field.
 *
 * <p>This command intentionally has no subsystem requirements so it can be run in parallel with
 * {@code AlignToTrench} while still respecting "subsystems not used by something else" before
 * starting the outtake.
 */
public class AutoSpitTrenchExit extends Command {

  private final DriveIO drive;
  private final IntakeRollersTalonFX intakeRollers;
  private final PreIndexer preIndexer;
  private final CommandXboxController driverController;
  private final CommandXboxController operatorController;

  private Command activeOuttake = null;

  public AutoSpitTrenchExit(
      DriveIO drive,
      IntakeRollersTalonFX intakeRollers,
      PreIndexer preIndexer,
      CommandXboxController driverController,
      CommandXboxController operatorController) {
    this.drive = drive;
    this.intakeRollers = intakeRollers;
    this.preIndexer = preIndexer;
    this.driverController = driverController;
    this.operatorController = operatorController;
  }

  @Override
  public void initialize() {
    setName("Auto Spit (Trench Exit)");
  }

  @Override
  public void execute() {
    if (activeOuttake != null && !activeOuttake.isScheduled()) {
      activeOuttake = null;
    }

    if (activeOuttake == null && shouldStartSpitting()) {
      activeOuttake =
          new AutoSpitOuttake(drive, intakeRollers, preIndexer, driverController, operatorController)
              .withInterruptBehavior(InterruptionBehavior.kCancelSelf);
      CommandScheduler.getInstance().schedule(activeOuttake);
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (activeOuttake != null && activeOuttake.isScheduled()) {
      activeOuttake.cancel();
    }
    activeOuttake = null;
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private boolean shouldStartSpitting() {
    // Only when it is not our shift
    if (HubShiftUtil.getShiftedShiftInfo().active()) {
      return false;
    }

    // Only if the operator/driver isn't already commanding these subsystems via the default logic
    if (operatorOrDriverUsingIntakeOrIndexer(driverController, operatorController)) {
      return false;
    }

    // Only if nothing else is currently commanding the subsystems
    if (!subsystemsFreeForAutoSpit(intakeRollers, preIndexer)) {
      return false;
    }

    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    Pose2d pose = drive.getCurrentPose();

    if (!isInAllianceZone(pose, alliance)) {
      return false;
    }

    if (!isLeavingAllianceZone(drive, alliance)) {
      return false;
    }

    // Back of robot faces alliance wall -> robot faces away from alliance wall
    return isFacingAwayFromAlliance(pose, alliance);
  }

  private static boolean operatorOrDriverUsingIntakeOrIndexer(
      CommandXboxController driver, CommandXboxController operator) {
    return driver.leftTrigger().getAsBoolean()
        || driver.rightTrigger().getAsBoolean()
        || driver.a().getAsBoolean()
        || driver.y().getAsBoolean()
        || operator.leftTrigger().getAsBoolean()
        || operator.y().getAsBoolean()
        || operator.rightTrigger().getAsBoolean();
  }

  private static boolean subsystemsFreeForAutoSpit(
      IntakeRollersTalonFX intakeRollers, PreIndexer preIndexer) {
    Command requiringRollers = CommandScheduler.getInstance().requiring(intakeRollers);
    Command requiringPreIndexer = CommandScheduler.getInstance().requiring(preIndexer);

    Command rollersDefault = intakeRollers.getDefaultCommand();
    Command preIndexerDefault = preIndexer.getDefaultCommand();

    boolean rollersFree = requiringRollers == null || requiringRollers == rollersDefault;
    boolean preIndexerFree = requiringPreIndexer == null || requiringPreIndexer == preIndexerDefault;

    return rollersFree && preIndexerFree;
  }

  private static boolean isInAllianceZone(Pose2d pose, Alliance alliance) {
    double x = pose.getX();
    return alliance == Alliance.Red
        ? x > AlignmentConstants.RedAllianceZoneEnd.getX()
        : x < AlignmentConstants.BlueAllianceZoneEnd.getX();
  }

  private static boolean isLeavingAllianceZone(DriveIO drive, Alliance alliance) {
    ChassisSpeeds fieldSpeeds = drive.getCurrentSpeedsFieldRelative();
    return alliance == Alliance.Red
        ? fieldSpeeds.vxMetersPerSecond < -IntakeConstants.kAutoSpitMinExitSpeedMps
        : fieldSpeeds.vxMetersPerSecond > IntakeConstants.kAutoSpitMinExitSpeedMps;
  }

  private static boolean isFacingAwayFromAlliance(Pose2d pose, Alliance alliance) {
    double heading = pose.getRotation().getRadians();
    double desiredHeading = alliance == Alliance.Red ? Math.PI : 0.0;
    return Math.abs(MathUtil.angleModulus(heading - desiredHeading))
        < IntakeConstants.kAutoSpitHeadingTolRad;
  }

  private static final class AutoSpitOuttake extends Command {
    private final DriveIO drive;
    private final IntakeRollersTalonFX intakeRollers;
    private final PreIndexer preIndexer;
    private final CommandXboxController driverController;
    private final CommandXboxController operatorController;

    private double nextFuelEjectTimeSec = 0.0;

    private AutoSpitOuttake(
        DriveIO drive,
        IntakeRollersTalonFX intakeRollers,
        PreIndexer preIndexer,
        CommandXboxController driverController,
        CommandXboxController operatorController) {
      this.drive = drive;
      this.intakeRollers = intakeRollers;
      this.preIndexer = preIndexer;
      this.driverController = driverController;
      this.operatorController = operatorController;
      addRequirements(intakeRollers, preIndexer);
    }

    @Override
    public void initialize() {
      setName("Auto Spit Outtake");
      nextFuelEjectTimeSec = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
      intakeRollers.set(IntakeConstants.kAutoSpitOuttakeDutyCycle);
      preIndexer.set(IntakeConstants.kAutoSpitOuttakeDutyCycle);

      maybeEjectFuelInSim();
    }

    @Override
    public boolean isFinished() {
      return !shouldContinueSpitting();
    }

    @Override
    public void end(boolean interrupted) {
      intakeRollers.set(0.0);
      preIndexer.set(0.0);
    }

    private boolean shouldContinueSpitting() {
      if (HubShiftUtil.getShiftedShiftInfo().active()) {
        return false;
      }
      if (operatorOrDriverUsingIntakeOrIndexer(driverController, operatorController)) {
        return false;
      }

      Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
      Pose2d pose = drive.getCurrentPose();
      if (!isInAllianceZone(pose, alliance)) {
        return false;
      }
      if (!isLeavingAllianceZone(drive, alliance)) {
        return false;
      }
      return isFacingAwayFromAlliance(pose, alliance);
    }

    private Pose2d getPoseForFuelSim() {
      if (!Robot.isSimulation()) {
        return drive.getCurrentPose();
      }
      try {
        return ((SimDriveSubsystem) drive).getRealPoseSim();
      } catch (Exception e) {
        return drive.getCurrentPose();
      }
    }

    private void maybeEjectFuelInSim() {
      if (!Robot.isSimulation()) {
        return;
      }

      double now = Timer.getFPGATimestamp();
      if (now < nextFuelEjectTimeSec) {
        return;
      }

      if (preIndexer.getBalls() <= 0) {
        nextFuelEjectTimeSec = now + IntakeConstants.kAutoSpitBallIntervalSec;
        return;
      }

      Pose2d robotPose = getPoseForFuelSim();
      ChassisSpeeds fieldSpeeds = drive.getCurrentSpeedsFieldRelative();

      // Spawn just behind the registered intake bbox to avoid immediately re-intaking.
      double fuelRadius = FieldConstants.fuelDiameter / 2.0;
      double intakeXMin = -(0.635 + 2 * (IntakeConstants.kIntakeLengthMeters * 0.9)) / 2.0;
      double intakeYMin = -(0.6) / 2.0;
      double intakeYMax = (0.65) / 2.0;
      double xSpawnRobot = intakeXMin - fuelRadius - 0.02;
      double ySpawnRobot = (intakeYMin + intakeYMax) / 2.0;

      Pose2d spawnPose2d =
          robotPose.transformBy(new Transform2d(xSpawnRobot, ySpawnRobot, new Rotation2d()));
      Translation3d initialPosition =
          new Translation3d(spawnPose2d.getX(), spawnPose2d.getY(), fuelRadius);

      double theta = robotPose.getRotation().getRadians();
      double spitVx = IntakeConstants.kAutoSpitEjectSpeedMps * Math.cos(theta + Math.PI);
      double spitVy = IntakeConstants.kAutoSpitEjectSpeedMps * Math.sin(theta + Math.PI);
      Translation3d initialVelocity =
          new Translation3d(
              fieldSpeeds.vxMetersPerSecond + spitVx,
              fieldSpeeds.vyMetersPerSecond + spitVy,
              0.0);

      boolean spawned = FuelSim.getInstance().spawnFuelIfAvailable(initialPosition, initialVelocity);
      if (spawned) {
        preIndexer.removeBall();
      }

      nextFuelEjectTimeSec = now + IntakeConstants.kAutoSpitBallIntervalSec;
    }
  }
}

