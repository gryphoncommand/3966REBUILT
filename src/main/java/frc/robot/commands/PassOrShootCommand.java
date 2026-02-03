package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AlignmentConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Flywheel.FlywheelIO;
import frc.robot.subsystems.Hood.HoodIO;

/**
 * Wrapper command that chooses between passing to the ally zone (when past the alliance
 * zone end) or shooting into the hub. The decision mirrors the inline ConditionalCommand
 * previously in RobotContainer.
 */
public class PassOrShootCommand extends Command {

    private ConditionalCommand delegate;
    private ParallelCommandGroup passGroup;
    private ParallelCommandGroup shootGroup;
    

    public PassOrShootCommand(DriveSubsystem drive, CommandXboxController driverController, HoodIO hood, FlywheelIO flywheel) {
        // declare requirements so scheduling conflicts are avoided
        addRequirements(drive, hood, flywheel);

        // choose depot vs outpost align command by comparing current Y
        ConditionalCommand choosePassingAlign = new ConditionalCommand(
            new AlignToGoal(drive, driverController, AlignmentConstants.PassingPoseDepot, true),
            new AlignToGoal(drive, driverController, AlignmentConstants.PassingPoseOutpost, true),
            () -> Math.abs(drive.getCurrentPose().getY() - AlignmentConstants.PassingPoseDepot.getY()) < Math.abs(drive.getCurrentPose().getY() - AlignmentConstants.PassingPoseOutpost.getY())
        );

        ConditionalCommand choosePassingShot = new ConditionalCommand(
            new PreparePassSOTM(hood, flywheel, drive, ShooterConstants.FakePassingValues, AlignmentConstants.PassingPoseDepot),
            new PreparePassSOTM(hood, flywheel, drive, ShooterConstants.FakePassingValues, AlignmentConstants.PassingPoseOutpost),
            () -> Math.abs(drive.getCurrentPose().getY() - AlignmentConstants.PassingPoseDepot.getY()) < Math.abs(drive.getCurrentPose().getY() - AlignmentConstants.PassingPoseOutpost.getY())
        );

        passGroup = new ParallelCommandGroup(
            choosePassingAlign,
            choosePassingShot
        );

        shootGroup = new ParallelCommandGroup(
            new AlignToGoal(drive, driverController, DriverStation.getAlliance().get() == Alliance.Red ? AlignmentConstants.RedHubPose : AlignmentConstants.BlueHubPose, true),
            new PrepareSOTM(hood, flywheel, drive, ShooterConstants.FakeShootingValues)
        );

        // condition: are we past the alliance zone end?
        delegate = new ConditionalCommand(
            passGroup,
            shootGroup,
            () -> {
                double x = drive.getCurrentPose().getX();
                return DriverStation.getAlliance().get() == Alliance.Red ? x < AlignmentConstants.RedAllianceZoneEnd.getX() : x > AlignmentConstants.BlueAllianceZoneEnd.getX();
            }
        );
    }

    @Override
    public void initialize() {
        delegate.initialize();
    }

    @Override
    public void execute() {
        // condition: are we past the alliance zone end?
        delegate.execute();
    }

    @Override
    public boolean isFinished() {
        return delegate.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        delegate.end(interrupted);
    }
}
