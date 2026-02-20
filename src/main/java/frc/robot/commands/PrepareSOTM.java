package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hood.HoodIO;
import frc.robot.Constants.AlignmentConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Flywheel.FlywheelIO;
import frc.GryphonLib.ShooterState;
import frc.GryphonLib.ShooterInterpolator;

import static edu.wpi.first.units.Units.Seconds;

import java.util.List;

import org.photonvision.PhotonUtils;

public class PrepareSOTM extends Command {

    private final HoodIO hood;
    private final FlywheelIO flywheel;
    private final DriveSubsystem driveData;
    private final List<ShooterState> table;
    private Pose2d effectiveGoalPose;

    public PrepareSOTM(
            HoodIO hood,
            FlywheelIO flywheel,
            DriveSubsystem driveData,
            List<ShooterState> table) {

        this.hood = hood;
        this.flywheel = flywheel;
        this.driveData = driveData;
        this.table = table;

        addRequirements(hood, flywheel);
        SmartDashboard.putBoolean("SOTM Goal Calculating", false);
        setName("Prepare SOTM");
    }

    @Override
    public void execute() {
        SmartDashboard.putBoolean("SOTM Goal Calculating", true);

        Pose2d hubPose = AlignmentConstants.HubPose;

        ChassisSpeeds shotMovement = driveData.getCurrentSpeedsFieldRelative();

        Pose2d shooterPose = driveData.getCurrentPose().plus(ShooterConstants.kRobotToShooter);

        shotMovement.vxMetersPerSecond = -shotMovement.vxMetersPerSecond;
        shotMovement.vyMetersPerSecond = -shotMovement.vyMetersPerSecond;
        shotMovement.omegaRadiansPerSecond = 0;

        double distanceToGoal = PhotonUtils.getDistanceToPose(shooterPose, hubPose);
        
        ShooterState state = ShooterInterpolator.interpolate(
                table, distanceToGoal);

        for (int i = 0; i < 5; i++) {
            var timeOfFlight = Seconds.of(state.flightTimeSec());
            effectiveGoalPose = hubPose.exp(shotMovement.toTwist2d(timeOfFlight.in(Seconds)));
            distanceToGoal = PhotonUtils.getDistanceToPose(shooterPose, effectiveGoalPose);
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
