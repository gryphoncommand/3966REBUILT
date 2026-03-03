package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climber.ClimberIO;

public class DeployClimber extends Command {
    private ClimberIO climber;

    public DeployClimber(ClimberIO climber){
        this.climber = climber;
        addRequirements(climber);
    }

    @Override
    public void initialize() {
        climber.setPosition(ClimberConstants.kFullUpPosition);
    }

    @Override
    public void end(boolean interrupted) {
        climber.setPosition(0);
    }
}
