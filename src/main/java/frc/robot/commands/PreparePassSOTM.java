package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hood.HoodIO;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Flywheel.FlywheelIO;
import frc.GryphonLib.ShooterState;
import frc.GryphonLib.ShooterInterpolator;

import static edu.wpi.first.units.Units.Seconds;

import java.util.List;

public class PreparePassSOTM extends Command {

    private final HoodIO hood;
    private final FlywheelIO flywheel;
    private final DriveSubsystem driveData;
    private final List<ShooterState> table;
    private Pose2d effectiveGoalPose;
    private Pose2d goalPose;

    public PreparePassSOTM(
            HoodIO hood,
            FlywheelIO flywheel,
            DriveSubsystem driveData,
            List<ShooterState> table,
            Pose2d goalPose) {

        this.hood = hood;
        this.flywheel = flywheel;
        this.driveData = driveData;
        this.table = table;
        this.goalPose = goalPose;

        addRequirements(hood.returnSubsystem(),
                flywheel.returnSubsystem());
        SmartDashboard.putBoolean("SOTM Goal Calculating", false);
    }

    @Override
    public void execute() {
        SmartDashboard.putBoolean("SOTM Goal Calculating", true);

        ChassisSpeeds shotMovement = driveData.getCurrentSpeedsFieldRelative();

        shotMovement.vxMetersPerSecond = -shotMovement.vxMetersPerSecond;
        shotMovement.vyMetersPerSecond = -shotMovement.vyMetersPerSecond;
        shotMovement.omegaRadiansPerSecond = 0;

        double distanceToGoal = driveData.getDistanceToPose(goalPose);
        
        ShooterState state = ShooterInterpolator.interpolate(
                table, distanceToGoal);

        for (int i = 0; i < 20; i++) {
            var timeOfFlight = Seconds.of(state.flightTimeSec());
            effectiveGoalPose = goalPose.exp(shotMovement.toTwist2d(timeOfFlight.in(Seconds)));
            distanceToGoal = driveData.getDistanceToPose(effectiveGoalPose);
            state = ShooterInterpolator.interpolate(
                table, distanceToGoal);
        }

        driveData.getField().getObject("SOTM Goal").setPose(effectiveGoalPose);
        

        hood.setAngle(state.hoodAngleDeg());
        flywheel.setVelocity(state.flywheelRPM());
    }

    @Override
    public boolean isFinished() {
        return false;
        // return hood.atTarget(3.0) && flywheel.atTarget(50);
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("SOTM Goal Calculating", false);
    }
}
