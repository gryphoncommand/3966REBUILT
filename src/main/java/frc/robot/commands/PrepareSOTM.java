package frc.robot.commands;

    import edu.wpi.first.math.geometry.Pose2d;
    import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
    import edu.wpi.first.units.measure.Time;
    import edu.wpi.first.wpilibj.DriverStation;
    import edu.wpi.first.wpilibj.DriverStation.Alliance;
    import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
    import edu.wpi.first.wpilibj2.command.Command;
    import frc.robot.subsystems.Hood.HoodIO;
    import frc.robot.Constants.AlignmentConstants;
    import frc.robot.Robot;
    import frc.robot.subsystems.DriveSubsystem;
    import frc.robot.subsystems.Flywheel.FlywheelIO;
    import frc.GryphonLib.ShooterState;
    import frc.GryphonLib.ShooterInterpolator;

import static edu.wpi.first.units.Units.Seconds;

import java.util.List;

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

        addRequirements(hood.returnSubsystem(),
                flywheel.returnSubsystem());
        SmartDashboard.putBoolean("SOTM Goal Calculating", false);
    }

    @Override
    public void execute() {
        SmartDashboard.putBoolean("SOTM Goal Calculating", true);

        // shot time from the interpolator
        Time shotTime = Seconds.of(1.0);

        Pose2d hubPose = DriverStation.getAlliance().get() == Alliance.Red ? AlignmentConstants.RedHubPose
                : AlignmentConstants.BlueHubPose;

        ChassisSpeeds shotMovement = ChassisSpeeds.fromRobotRelativeSpeeds(driveData.getCurrentSpeeds(),
                Rotation2d.fromDegrees(Robot.isReal() ? driveData.getHeading()
                        : driveData.getCurrentPose().getRotation().getDegrees()));
        shotMovement.vxMetersPerSecond = -shotMovement.vxMetersPerSecond;
        shotMovement.vyMetersPerSecond = -shotMovement.vyMetersPerSecond;
        shotMovement.omegaRadiansPerSecond = 0;

        effectiveGoalPose = hubPose.exp(shotMovement.toTwist2d(shotTime.in(Seconds)));
        driveData.getField().getObject("SOTM Goal").setPose(effectiveGoalPose);

        double distanceToGoal = driveData.getDistanceToPose(effectiveGoalPose);
        
        ShooterState state = ShooterInterpolator.interpolate(
                table, distanceToGoal);

        for (int i = 0; i < 20; i++) {
            var timeOfFlight = state.flightTimeSec();
            double offsetX = shotMovement.vxMetersPerSecond * timeOfFlight;
            double offsetY = shotMovement.vyMetersPerSecond * timeOfFlight;
            Pose2d lookaheadPose = new Pose2d(
                    driveData.getCurrentPose().getTranslation().plus(new Translation2d(offsetX, offsetY)),
                    driveData.getCurrentPose().getRotation());
            distanceToGoal = driveData.getDistanceToPose(lookaheadPose);
            state = ShooterInterpolator.interpolate(
                table, distanceToGoal);
        }
        

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
