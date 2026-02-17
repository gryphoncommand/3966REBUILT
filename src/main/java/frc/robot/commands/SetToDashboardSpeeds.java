package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Flywheel.FlywheelIO;
import frc.robot.subsystems.Hood.HoodIO;

public class SetToDashboardSpeeds extends Command {
    private HoodIO hood;
    private FlywheelIO flywheel;
    private Timer rechargeTimer = new Timer();
    private boolean reachedTarget = false;

    public SetToDashboardSpeeds(HoodIO hood, FlywheelIO flywheel){
        this.hood = hood;
        this.flywheel = flywheel;

        addRequirements(hood, flywheel);

        SmartDashboard.putNumber("Flywheel Manual Speed", 2000);
        SmartDashboard.putNumber("Hood Manual Angle", 45);
    }

    @Override
    public void initialize() {
        flywheel.setVelocity(SmartDashboard.getNumber("Flywheel Manual Speed", 2000));
        hood.setAngle(SmartDashboard.getNumber("Hood Manual Angle", 45));
        SmartDashboard.putBoolean("Manual Shoot Ready", hood.atTarget(1.0) && flywheel.atTarget(30));
        rechargeTimer.restart();
    }

    @Override
    public void execute() {
        if (flywheel.atTarget(50) && reachedTarget == false){
            reachedTarget = true;
            Logger.recordOutput("Recharge Time", String.valueOf(rechargeTimer.get()));
        } else {
            if (!flywheel.atTarget(50) && reachedTarget == true){
                reachedTarget = false;
                rechargeTimer.reset();
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        flywheel.set(0);
        hood.stow();
    }
}
