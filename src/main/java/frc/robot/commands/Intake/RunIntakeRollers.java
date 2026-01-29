package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake.IntakeRollersTalonFX;

public class RunIntakeRollers extends Command {
    IntakeRollersTalonFX intakeRollers;

    public RunIntakeRollers(IntakeRollersTalonFX intakeRollers){
        this.intakeRollers = intakeRollers;
        addRequirements(intakeRollers);
    }

    @Override
    public void initialize() {
        setName("Intaking");
        intakeRollers.setVelocity(IntakeConstants.kIntakeSpeedRPM);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intakeRollers.setVelocity(0);
    }
}
