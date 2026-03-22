package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AlignmentConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Flywheel.FlywheelIO;

public class PassCommand extends Command {
    private ParallelCommandGroup passGroup;
    private DriveSubsystem drive;
    private boolean depot;

    public PassCommand(DriveSubsystem drive, CommandXboxController driverController, FlywheelIO flywheel){
        this.drive = drive;
        addRequirements(drive, flywheel);
        depot = Math.abs(drive.getCurrentPose().getY() - AlignmentConstants.PassingPoseDepot.getY()) < Math.abs(drive.getCurrentPose().getY() - AlignmentConstants.PassingPoseOutpost.getY());
        
        // choose depot vs outpost align command by comparing current Y
        ConditionalCommand choosePassingAlign = new ConditionalCommand(
            new AlignToGoal(drive, driverController, AlignmentConstants.PassingPoseDepot, true),
            new AlignToGoal(drive, driverController, AlignmentConstants.PassingPoseOutpost, true),
            () -> depot
        );

        ConditionalCommand choosePassingShot = new ConditionalCommand(
            new PrepareSOTM(flywheel, drive, AlignmentConstants.PassingPoseDepot, ShooterConstants.RealPassingValues),
            new PrepareSOTM(flywheel, drive, AlignmentConstants.PassingPoseOutpost, ShooterConstants.RealPassingValues),
            () -> depot
        );

        passGroup = new ParallelCommandGroup(
            choosePassingAlign,
            choosePassingShot
        );
    }

    @Override
    public void initialize() {
        setName("Prepare Pass");
        passGroup.initialize();
    }

    @Override
    public void execute() {
        passGroup.execute();
    }

    @Override
    public boolean isFinished() {
        Pose2d currentPose = drive.getCurrentPose();
        if (isInMidfieldBlockZone(currentPose)) {
            return true;
        }
        Pose2d futurePose = currentPose.exp(drive.getCurrentSpeeds().toTwist2d(0.5));
        boolean changed = (Math.abs(futurePose.getY() - AlignmentConstants.PassingPoseDepot.getY()) < Math.abs(futurePose.getY() - AlignmentConstants.PassingPoseOutpost.getY()) != depot);
        return passGroup.isFinished() || changed;
    }

    @Override
    public void end(boolean interrupted) {
        passGroup.end(interrupted);
    }

    private boolean isInMidfieldBlockZone(Pose2d pose) {
        return Math.abs(pose.getY() - AlignmentConstants.kMidFieldY)
                < AlignmentConstants.kMidFieldHubBlockWidth / 2.0;
    }
}
