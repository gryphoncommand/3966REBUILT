package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.AlignmentConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Flywheel.FlywheelIO;
import frc.robot.subsystems.Hood.HoodIO;

public class SOTMDuringAuto extends Command {
    private ConditionalCommand delegate;
    private ParallelCommandGroup passGroup;
    private ParallelCommandGroup shootGroup;
    private boolean inAllianceZone;
    DriveSubsystem drive;
    HoodIO hood;
    FlywheelIO flywheel;

    public SOTMDuringAuto(DriveSubsystem drive, HoodIO hood, FlywheelIO flywheel){
        this.drive = drive;
        this.hood = hood;
        this.flywheel = flywheel;

        addRequirements(hood, flywheel);

        // choose depot vs outpost align command by comparing current Y
        ConditionalCommand choosePassingAlign = new ConditionalCommand(
                new AlignToGoalAuto(drive, AlignmentConstants.PassingPoseDepot, true),
                new AlignToGoalAuto(drive, AlignmentConstants.PassingPoseOutpost, true),
                () -> Math.abs(drive.getCurrentPose().getY() - AlignmentConstants.PassingPoseDepot.getY()) < Math
                        .abs(drive.getCurrentPose().getY() - AlignmentConstants.PassingPoseOutpost.getY()));

        ConditionalCommand choosePassingShot = new ConditionalCommand(
                new PreparePassSOTM(hood, flywheel, drive, ShooterConstants.RealPassingValues,
                        AlignmentConstants.PassingPoseDepot),
                new PreparePassSOTM(hood, flywheel, drive, ShooterConstants.RealPassingValues,
                        AlignmentConstants.PassingPoseOutpost),
                () -> Math.abs(drive.getCurrentPose().getY() - AlignmentConstants.PassingPoseDepot.getY()) < Math
                        .abs(drive.getCurrentPose().getY() - AlignmentConstants.PassingPoseOutpost.getY()));

        passGroup = new ParallelCommandGroup(
                choosePassingAlign,
                choosePassingShot);

        shootGroup = new ParallelCommandGroup(
                new AlignToGoalAuto(drive, AlignmentConstants.HubPose, true),
                new PrepareSOTM(hood, flywheel, drive, ShooterConstants.RealShootingValues));

        // condition: are we past the alliance zone end?
        delegate = new ConditionalCommand(
                passGroup,
                shootGroup,
                () -> {
                    Pose2d futurePose = drive.getCurrentPose().exp(drive.getCurrentSpeeds().toTwist2d(0.5));
                    double x = futurePose.getX();
                    return DriverStation.getAlliance().get() == Alliance.Red
                            ? x < AlignmentConstants.RedAllianceZoneEnd.getX()
                            : x > AlignmentConstants.BlueAllianceZoneEnd.getX();
                });

        Pose2d futurePose = drive.getCurrentPose().exp(drive.getCurrentSpeeds().toTwist2d(0.5));
        double x = futurePose.getX();
        inAllianceZone = DriverStation.getAlliance().get() == Alliance.Red
                ? x < AlignmentConstants.RedAllianceZoneEnd.getX()
                : x > AlignmentConstants.BlueAllianceZoneEnd.getX();
    }

    @Override
    public void initialize() {
        delegate.initialize();
    }

    @Override
    public void execute() {
        delegate.execute();
    }

    @Override
    public boolean isFinished() {
        Pose2d futurePose = drive.getCurrentPose().exp(drive.getCurrentSpeeds().toTwist2d(0.5));
        double x = futurePose.getX();
        boolean inAllianceNow = DriverStation.getAlliance().get() == Alliance.Red ? x < AlignmentConstants.RedAllianceZoneEnd.getX() : x > AlignmentConstants.BlueAllianceZoneEnd.getX();
        return delegate.isFinished() || inAllianceNow != inAllianceZone;
    }

    @Override
    public void end(boolean interrupted) {
        delegate.end(interrupted);
    }
}
