package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.GryphonLib.MovementCalculations;
import frc.GryphonLib.PositionCalculations;
import frc.robot.Robot;
import frc.robot.Constants.AlignmentConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Drive.DriveIO;

public class AlignToGoal extends Command {
    private final DriveIO drive;
    private final PIDController turnPID = AlignmentConstants.turnPID;
    private final CommandXboxController controller;
    private Pose2d goalPose;
    private double yawError;
    private Debouncer alignDebouncer = new Debouncer(0.2);
    private boolean SOTM;
    private Transform2d kRobotToShooter;

    public AlignToGoal(DriveIO drive, CommandXboxController controller, Pose2d goalPose, boolean SOTM) {
        this.drive = drive;
        this.controller = controller;
        this.goalPose = goalPose;
        this.SOTM = SOTM;

        kRobotToShooter = Robot.isReal() ? ShooterConstants.kRobotToShooter : new Transform2d(ShooterConstants.kRobotToShooter.getX(), ShooterConstants.kRobotToShooter.getY(), new Rotation2d(Math.PI/2));

        addRequirements(drive);
        if (SOTM && SmartDashboard.getBoolean("SOTM Goal Calculating", false)) {
            try {
                goalPose = drive.getField().getObject("SOTM Goal").getPose();
            } catch (Exception e) {}
        }

        yawError = PositionCalculations.getYawChangeToPose(
            drive.getCurrentPose().transformBy(kRobotToShooter),
            goalPose
        );
    }

    @Override
    public void initialize() {
        turnPID.reset();
        turnPID.setSetpoint(0.0);
    }

    @Override
    public void execute() {
        if (SOTM && SmartDashboard.getBoolean("SOTM Goal Calculating", false)) {
            try {
                goalPose = drive.getField().getObject("SOTM Goal").getPose();
            } catch (Exception e) {}
        }

        // Driver translation inputs
        double forward = -MathUtil.applyDeadband(controller.getLeftY(), OIConstants.kDriveDeadband);
        double strafe  = -MathUtil.applyDeadband(controller.getLeftX(), OIConstants.kDriveDeadband);

        // Compute yaw error from shooter pose instead of robot center
        yawError = PositionCalculations.getYawChangeToPose(
            drive.getCurrentPose().transformBy(kRobotToShooter),
            goalPose
        );
        SmartDashboard.putNumber("Yaw Align Error", Units.radiansToDegrees(yawError));

        // PID output
        double turn = turnPID.calculate(yawError);
        turn = MathUtil.clamp(turn, -1.0, 1.0);

        // Drive with driver’s translation + auto-turn
        drive.drive(forward, strafe, -turn, true);

        boolean withinAngleTol = Math.abs(yawError) < AlignmentConstants.ANGLE_TOLERANCE_RAD;
        boolean slowEnoughRot = Math.abs(drive.getCurrentSpeeds().omegaRadiansPerSecond) < AlignmentConstants.ANG_VEL_TOLERANCE_RAD_PER_SEC;
        boolean slowEnoughTrans = Math.abs(MovementCalculations.getVelocityMagnitude(drive.getCurrentSpeeds()).in(MetersPerSecond)) < AlignmentConstants.SPEED_VEL_TOLERANCE;

        if (SOTM) {
            withinAngleTol = Math.abs(yawError) < AlignmentConstants.SOTM_ANGLE_TOLERANCE_RAD;
            slowEnoughRot = true;
            slowEnoughTrans = true;
        }

        // Debouncer ensures it's stable for required time
        boolean alignedNow = alignDebouncer.calculate(withinAngleTol && slowEnoughRot && slowEnoughTrans);
        drive.setAlign(alignedNow);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(0.0, 0.0, 0.0, true);
        drive.setAlign(false);
    }
}
