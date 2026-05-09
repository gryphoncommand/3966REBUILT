package frc.robot.AI;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.*;
import com.pathplanner.lib.pathfinding.Pathfinder;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

public class IsolatedPathfindingCommand extends Command {
    private final Timer timer = new Timer();
    private final Pathfinder pathfinder; // per-bot instance, NOT the global singleton
    private final Pose2d targetPose;
    private final GoalEndState goalEndState;
    private final PathConstraints constraints;
    private final Supplier<Pose2d> poseSupplier;
    private final Supplier<ChassisSpeeds> speedsSupplier;
    private final BiConsumer<ChassisSpeeds, DriveFeedforwards> output;
    private final PPHolonomicDriveController controller;
    private final RobotConfig robotConfig;

    private PathPlannerPath currentPath;
    private PathPlannerTrajectory currentTrajectory;
    private double timeOffset = 0;
    private boolean finish = false;

    public IsolatedPathfindingCommand(
            Pathfinder pathfinder,
            Pose2d targetPose,
            PathConstraints constraints,
            double goalEndVel,
            Supplier<Pose2d> poseSupplier,
            Supplier<ChassisSpeeds> speedsSupplier,
            BiConsumer<ChassisSpeeds, DriveFeedforwards> output,
            PPHolonomicDriveController controller,
            RobotConfig robotConfig,
            Subsystem... requirements) {
        addRequirements(requirements);
        this.pathfinder = pathfinder;
        this.targetPose = targetPose;
        this.goalEndState = new GoalEndState(goalEndVel, targetPose.getRotation());
        this.constraints = constraints;
        this.poseSupplier = poseSupplier;
        this.speedsSupplier = speedsSupplier;
        this.output = output;
        this.controller = controller;
        this.robotConfig = robotConfig;
    }

    @Override
    public void initialize() {
        currentTrajectory = null;
        timeOffset = 0;
        finish = false;

        Pose2d currentPose = poseSupplier.get();
        controller.reset(currentPose, speedsSupplier.get());

        if (currentPose.getTranslation().getDistance(targetPose.getTranslation()) < 0.5) {
            output.accept(new ChassisSpeeds(), DriveFeedforwards.zeros(robotConfig.numModules));
            finish = true;
        } else {
            pathfinder.setStartPosition(currentPose.getTranslation());
            pathfinder.setGoalPosition(targetPose.getTranslation());
        }
    }

    @Override
    public void execute() {
        if (finish) return;

        Pose2d currentPose = poseSupplier.get();
        ChassisSpeeds currentSpeeds = speedsSupplier.get();

        boolean skipUpdates = currentTrajectory != null &&
            currentPose.getTranslation().getDistance(
                currentTrajectory.getEndState().pose.getTranslation()) < 2.0;

        if (!skipUpdates && pathfinder.isNewPathAvailable()) {
            currentPath = pathfinder.getCurrentPath(constraints, goalEndState);

            if (currentPath != null) {
                currentTrajectory = new PathPlannerTrajectory(
                    currentPath, currentSpeeds, currentPose.getRotation(), robotConfig);

                if (!Double.isFinite(currentTrajectory.getTotalTimeSeconds())) {
                    finish = true;
                    return;
                }

                // Interpolate time offset so we don't start from t=0 if the robot
                // has already moved past the beginning of the new path
                int idx1 = 0, idx2 = 1;
                while (idx2 < currentTrajectory.getStates().size() - 1) {
                    double d2 = currentTrajectory.getState(idx2).pose.getTranslation()
                        .getDistance(currentPose.getTranslation());
                    double dn = currentTrajectory.getState(idx2 + 1).pose.getTranslation()
                        .getDistance(currentPose.getTranslation());
                    if (dn < d2) { idx1++; idx2++; } else break;
                }

                var s1 = currentTrajectory.getState(idx1);
                var s2 = currentTrajectory.getState(idx2);
                double segLen = s1.pose.getTranslation().getDistance(s2.pose.getTranslation());
                double t = segLen > 0 ? MathUtil.clamp(
                    currentPose.getTranslation().getDistance(s1.pose.getTranslation()) / segLen, 0, 1) : 0;
                timeOffset = MathUtil.interpolate(s1.timeSeconds, s2.timeSeconds, t);

                if (timeOffset <= 0.02 &&
                    Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond) < 0.1) {
                    timeOffset = 0.02;
                }
            }

            timer.reset();
            timer.start();
        }

        if (currentTrajectory != null) {
            var targetState = currentTrajectory.sample(timer.get() + timeOffset);
            output.accept(
                controller.calculateRobotRelativeSpeeds(currentPose, targetState),
                targetState.feedforwards);
        }
    }

    @Override
    public boolean isFinished() {
        if (finish) return true;
        if (currentTrajectory != null) {
            return timer.hasElapsed(currentTrajectory.getTotalTimeSeconds() - timeOffset);
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        if (!interrupted && goalEndState.velocityMPS() < 0.1) {
            output.accept(new ChassisSpeeds(), DriveFeedforwards.zeros(robotConfig.numModules));
        }
    }
}