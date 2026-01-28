package frc.robot.commands.Intake;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake.IntakeDeployIO;

public class IntakeStow extends Command {
    IntakeDeployIO intake;

    public IntakeStow(IntakeDeployIO intake){
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        setName("Intake Stow");
        SmartDashboard.putBoolean("Intake Deployed", false);
        intake.setPosition(IntakeConstants.kIntakeStowAngle);
    }

    @Override
    public boolean isFinished() {
        return intake.atTarget(Units.degreesToRotations(30));
    }
}
