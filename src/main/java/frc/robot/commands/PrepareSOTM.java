    package frc.robot.commands;

    import edu.wpi.first.math.geometry.Pose2d;
    import edu.wpi.first.math.geometry.Rotation2d;
    import edu.wpi.first.math.kinematics.ChassisSpeeds;
    import edu.wpi.first.units.measure.Time;
    import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
    import edu.wpi.first.wpilibj2.command.Command;
    import frc.robot.subsystems.Hood.HoodIO;
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
    private Pose2d goalPose;
    private Pose2d effectiveGoalPose;

    public PrepareSOTM(
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

        addRequirements(hood, flywheel);
        SmartDashboard.putBoolean("SOTM Goal Calculating", false);
    }

    @Override
    public void execute() {
        SmartDashboard.putBoolean("SOTM Goal Calculating", true);

        // TODO: This is a placeholder, make it do a few iterations to converge on a shot time from the interpolator
        Time shotTime = Seconds.of(0.5);
        
        ChassisSpeeds shotMovement = ChassisSpeeds.fromRobotRelativeSpeeds(driveData.getCurrentSpeeds(), Rotation2d.fromDegrees(Robot.isReal() ? driveData.getHeading() : driveData.getCurrentPose().getRotation().getDegrees()));
        shotMovement.vxMetersPerSecond = -shotMovement.vxMetersPerSecond;
        shotMovement.vyMetersPerSecond = -shotMovement.vyMetersPerSecond;
        shotMovement.omegaRadiansPerSecond = 0;

        effectiveGoalPose = goalPose.exp(shotMovement.toTwist2d(shotTime.in(Seconds)));
        driveData.getField().getObject("SOTM Goal").setPose(effectiveGoalPose);

        double distanceToGoal = driveData.getDistanceToPose(effectiveGoalPose);

        ShooterState state =
            ShooterInterpolator.interpolate(
                table, distanceToGoal);

        hood.setAngle(state.hoodAngleDeg);
        flywheel.setVelocity(state.flywheelRPM);
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
