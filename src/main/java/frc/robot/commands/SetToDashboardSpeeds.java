package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Flywheel.FlywheelIO;

public class SetToDashboardSpeeds extends Command {
    private FlywheelIO flywheel;
    private Timer rechargeTimer = new Timer();
    private boolean reachedTarget = false;

    public SetToDashboardSpeeds(FlywheelIO flywheel){
        this.flywheel = flywheel;

        addRequirements(flywheel);

        SmartDashboard.putNumber("Flywheel Manual Speed", 2000);
    }

    @Override
    public void initialize() {
        double rpm = SmartDashboard.getNumber("Flywheel Manual Speed", 2000);
        flywheel.setRealTarget(rpm);
        flywheel.setVelocity(rpm);
        SmartDashboard.putBoolean("Manual Shoot Ready", flywheel.atTarget(30));
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
    }
}
