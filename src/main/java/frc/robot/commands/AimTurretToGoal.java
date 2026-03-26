package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.GryphonLib.PositionCalculations;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.Drive.DriveIO;
import frc.robot.subsystems.Turret.TurretIO;

public class AimTurretToGoal extends Command {
    private final DriveIO drive;
    private final TurretIO turret;
    private Pose2d goalPose;
    private final boolean useSOTMGoal;

    public AimTurretToGoal(DriveIO drive, TurretIO turret, Pose2d goalPose, boolean useSOTMGoal) {
        this.drive = drive;
        this.turret = turret;
        this.goalPose = goalPose;
        this.useSOTMGoal = useSOTMGoal;

        addRequirements(turret);
        setName("Aim Turret To Goal");
    }

    @Override
    public void execute() {
        if (useSOTMGoal && SmartDashboard.getBoolean("SOTM Goal Calculating", false)) {
            try {
                goalPose = drive.getField().getObject("SOTM Goal").getPose();
            } catch (Exception e) {
            }
        }

        Pose2d shooterPose = drive.getCurrentPose().transformBy(ShooterConstants.kRobotToShooter).exp(drive.getCurrentSpeeds().toTwist2d(0.15));
        double yawErrorRad = PositionCalculations.getYawChangeToPose(shooterPose, goalPose);

        double desiredAngleDeg = Units.radiansToDegrees(yawErrorRad);
        double wrappedAngleDeg = wrapToLimits(desiredAngleDeg);

        turret.setAngle(wrappedAngleDeg);

        Logger.recordOutput("Turret/Yaw Error (deg)", desiredAngleDeg);
        Logger.recordOutput("Turret/Requested Angle (deg)", wrappedAngleDeg);
        Logger.recordOutput("Turret/Desired Turret Angle", wrappedAngleDeg);        
    }

    private double wrapToLimits(double angleDeg) {
        double min = TurretConstants.kTurretMinAngleDeg;
        double max = TurretConstants.kTurretMaxAngleDeg;
        double range = max - min;
        if (range <= 0.0) {
            return MathUtil.clamp(angleDeg, min, max);
        }

        double wrapped = angleDeg;
        while (wrapped < min) {
            wrapped += range;
        }
        while (wrapped > max) {
            wrapped -= range;
        }

        double current = turret.getAngle();
        double best = wrapped;
        double bestError = Math.abs(wrapped - current);

        double plus = wrapped + range;
        if (plus >= min && plus <= max) {
            double error = Math.abs(plus - current);
            if (error < bestError) {
                best = plus;
                bestError = error;
            }
        }

        double minus = wrapped - range;
        if (minus >= min && minus <= max) {
            double error = Math.abs(minus - current);
            if (error < bestError) {
                best = minus;
            }
        }

        return MathUtil.clamp(best, min, max);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
