package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber.ClimberIO;

public class StowClimber extends Command {
    private ClimberIO climber;

    public StowClimber(ClimberIO climber){
        this.climber = climber;
        addRequirements(climber);
    }

    @Override
    public void initialize() {
        climber.setPosition(0);
    }
}
