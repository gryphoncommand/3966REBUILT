package frc.robot.commands.Intake;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake.IntakeDeployIO;

public class IntakeDeploy extends Command {
    IntakeDeployIO intake;

    public IntakeDeploy(IntakeDeployIO intake){
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        setName("Intake Deploy");
        intake.setPosition(IntakeConstants.kIntakeDeployAngle);
    }

    @Override
    public boolean isFinished() {
        return intake.atTarget(Units.degreesToRotations(30));
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted){
            SmartDashboard.putBoolean("Intake Deployed", true);
        }
    }
}
