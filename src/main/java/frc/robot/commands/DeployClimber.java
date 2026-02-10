package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber.ClimberIO;

public class DeployClimber extends Command {
    private ClimberIO climber;

    public DeployClimber(ClimberIO climber){
        this.climber = climber;
        addRequirements(climber);
    }

    @Override
    public void initialize() {
        climber.setPosition(Inches.of(5));
    }

    @Override
    public void end(boolean interrupted) {
        climber.setPosition(Inches.of(0));
    }
}
