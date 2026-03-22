package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Flywheel.FlywheelIO;

public class ResetShooter extends Command {
    private FlywheelIO flywheel;

    public ResetShooter(FlywheelIO flywheel){
        this.flywheel = flywheel;

        addRequirements(flywheel);
    }

    @Override
    public void initialize() {
        flywheel.setRealTarget(0);
        flywheel.set(0);
    }

    @Override
    public boolean isFinished() {
        return flywheel.atTarget(30);
    }
}
