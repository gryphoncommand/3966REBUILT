package frc.robot.commands;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class SOTMDuringAuto extends Command {
    DriveSubsystem drive;
    PathPlannerPath pathToFollow;


    public SOTMDuringAuto(DriveSubsystem drive, PathPlannerPath path){
        this.drive = drive;
        this.pathToFollow = path;

        addRequirements(drive);
    }

    @Override
    public void execute() {
    }
}
