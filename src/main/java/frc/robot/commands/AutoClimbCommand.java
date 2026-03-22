package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Comparator;
import java.util.List;
import java.util.Set;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.GryphonLib.MovementCalculations;
import frc.GryphonLib.AllianceFlipUtil;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Climber.ClimberIO;

public class AutoClimbCommand extends Command {

    private final Command sequence;

    public AutoClimbCommand(DriveSubsystem drive, ClimberIO climber) {

        addRequirements(drive, climber.returnSubsystem());

        Logger.recordOutput("Auto Climb Pre Climb", getNearestPreClimbPose(drive));
        Logger.recordOutput("Auto Climb Climb Pose", getNearestClimbPose(drive));

        sequence = Commands.sequence(

            Commands.defer(
                () -> drive.PathToPose(getNearestPreClimbPose(drive), 0)
                .andThen(
                new RunCommand(() ->
                    drive.drive(0.0, 0, 0, false),
                    drive
                ).until(()-> (MovementCalculations.getVelocityMagnitude(drive.getCurrentSpeeds()).in(MetersPerSecond) < 0.1))
            ),
                Set.of(drive)
            ),

            // Extend climber
            Commands.runOnce(() ->
                climber.setPosition(ClimberConstants.kFullUpPosition)
            ),

            // Drive to nearest (alliance-corrected) climb pose
            Commands.defer(
                () -> PositionPIDCommand.generateCommand(drive, getNearestClimbPose(drive), Seconds.of(1.5))
                .andThen(
                new RunCommand(() ->
                    drive.drive(0.0, 0, 0, false),
                    drive
                ).until(()-> (MovementCalculations.getVelocityMagnitude(drive.getCurrentSpeeds()).in(MetersPerSecond) < 0.1))
            ),
                Set.of(drive)
            ),

            Commands.waitUntil(() ->
                climber.atTarget(2.0)
            ),

            // Drive backwards robot-relative
            Commands.defer(() -> {
                return new RunCommand(() ->
                    drive.drive(-0.15, 0.005, 0, false),
                    drive
                ).withTimeout(1);
            }, Set.of(drive)),

            new InstantCommand(() ->
                    drive.drive(0.0, 0.0, 0, false),
                    drive
            ),

            // Retract climber
            Commands.runOnce(() ->
                climber.setPosition(0.0)
            ),

            Commands.waitUntil(() ->
                climber.atTarget(2.0)
            )
        );
    }

    /**
     * Returns the nearest climb pose, flipped automatically
     * for red alliance using AllianceFlipUtil.
     */
    private Pose2d getNearestPreClimbPose(DriveSubsystem drive) {

        Pose2d current = drive.getCurrentPose();

        List<Pose2d> climbPosesBlue = List.of(
            ClimberConstants.kLeftPreClimb,
            ClimberConstants.kRightPreClimb
        );

        List<Pose2d> allianceCorrected = climbPosesBlue.stream()
            .map(AllianceFlipUtil::apply)
            .toList();

        return allianceCorrected.stream()
            .min(Comparator.comparingDouble(p ->
                current.getTranslation().getDistance(p.getTranslation())))
            .orElse(AllianceFlipUtil.apply(ClimberConstants.kLeftClimbPose));
    }


    private Pose2d getNearestClimbPose(DriveSubsystem drive) {

        Pose2d current = drive.getCurrentPose();

        List<Pose2d> climbPosesBlue = List.of(
            ClimberConstants.kLeftClimbPose,
            ClimberConstants.kRightClimbPose
        );

        List<Pose2d> allianceCorrected = climbPosesBlue.stream()
            .map(AllianceFlipUtil::apply)
            .toList();

        return allianceCorrected.stream()
            .min(Comparator.comparingDouble(p ->
                current.getTranslation().getDistance(p.getTranslation())))
            .orElse(AllianceFlipUtil.apply(ClimberConstants.kLeftClimbPose));
    }

    @Override
    public void initialize() {
        sequence.initialize();
    }

    @Override
    public void execute() {
        sequence.execute();
    }

    @Override
    public void end(boolean interrupted) {
        sequence.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return sequence.isFinished();
    }
}