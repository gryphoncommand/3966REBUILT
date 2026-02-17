package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Flywheel.FlywheelIO;
import frc.robot.subsystems.Hood.HoodIO;

public class ResetShooter extends Command {
    private HoodIO hood;
    private FlywheelIO flywheel;

    public ResetShooter(HoodIO hood, FlywheelIO flywheel){
        this.hood = hood;
        this.flywheel = flywheel;

        addRequirements(hood, flywheel);
    }

    @Override
    public void initialize() {
        hood.stow();
        flywheel.set(0);
    }

    @Override
    public boolean isFinished() {
        return hood.atTarget(1.0) && flywheel.atTarget(30);
    }
}
