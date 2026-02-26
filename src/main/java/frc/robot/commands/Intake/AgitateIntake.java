package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake.IntakeDeployIO;

public class AgitateIntake extends Command {
    IntakeDeployIO intake;
    boolean agitateAngle;

    public AgitateIntake(IntakeDeployIO intake){
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setPosition(IntakeConstants.kIntakeAgitateAngle);
        agitateAngle = true;
    }

    @Override
    public void execute() {
        if (intake.atTarget(0.05)){
            if (agitateAngle){
                intake.setPosition(IntakeConstants.kIntakeAgitateAngle);
            } else {
                intake.setPosition(IntakeConstants.kIntakeDeployAngle);
            }

            agitateAngle = !agitateAngle;
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intake.setPosition(IntakeConstants.kIntakeDeployAngle);
    }
}
