package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Flywheel.FlywheelIO;
import frc.GryphonLib.ShooterState;
import frc.GryphonLib.ChassisAccelerations;
import frc.GryphonLib.ShooterInterpolator;
import frc.littletonUtils.LoggedTunableNumber;

import static edu.wpi.first.units.Units.Seconds;

import java.util.List;

import org.photonvision.PhotonUtils;

public class PrepareSOTM extends Command {

    private final FlywheelIO flywheel;
    private final DriveSubsystem driveData;
    private final List<ShooterState> table;
    private final Pose2d goalPose;
    private Pose2d effectiveGoalPose;
    private LoggedTunableNumber kShootDelay = new LoggedTunableNumber("SOTM/Shooter Shoot Delay", ShooterConstants.kShootDelay);
    private LoggedTunableNumber kPhaseDelay = new LoggedTunableNumber("SOTM/Shooter Phase Delay", ShooterConstants.kPhaseDelay);
    private LoggedTunableNumber kRPMChange = new LoggedTunableNumber("SOTM/Static Flywheel RPM Change", -130);
    private LoggedTunableNumber kTOFChange = new LoggedTunableNumber("SOTM/Static TOF Change", 0);
    private LoggedTunableNumber kFlywheelSetpointOffset = new LoggedTunableNumber("SOTM/Flywheel Recharge Compensation", ShooterConstants.kFlywheelRPMOffset);



    public PrepareSOTM(
            FlywheelIO flywheel,
            DriveSubsystem driveData,
            Pose2d goalPose,
            List<ShooterState> table) {

        this.flywheel = flywheel;
        this.driveData = driveData;
        this.table = table;
        this.goalPose = goalPose;

        addRequirements(flywheel);
        SmartDashboard.putBoolean("SOTM Goal Calculating", false);
        setName("Prepare SOTM");
    }

    @Override
    public void execute() {
        SmartDashboard.putBoolean("SOTM Goal Calculating", true);

        ChassisSpeeds shotMovement = driveData.getCurrentSpeedsFieldRelative();

        Pose2d shooterPose = driveData.getCurrentPose().exp(shotMovement.toTwist2d(kPhaseDelay.get())).plus(ShooterConstants.kRobotToShooter);

        ChassisAccelerations accel = driveData.getAcceleration();
        double shotTime = kPhaseDelay.get();
        if (ShooterConstants.accountForAccel){
            shotMovement.vxMetersPerSecond = -(shotMovement.vxMetersPerSecond + accel.getX() * shotTime);
            shotMovement.vyMetersPerSecond = -(shotMovement.vyMetersPerSecond + accel.getY() * shotTime);
            shotMovement.omegaRadiansPerSecond = 0;
        } else {
            shotMovement.vxMetersPerSecond = -shotMovement.vxMetersPerSecond;
            shotMovement.vyMetersPerSecond = -shotMovement.vyMetersPerSecond;
            shotMovement.omegaRadiansPerSecond = 0;
        }
        

        double distanceToGoal = PhotonUtils.getDistanceToPose(shooterPose, goalPose);
        
        ShooterState state = ShooterInterpolator.interpolate(
                table, distanceToGoal);

        for (int i = 0; i < 15; i++) {
            var timeOfFlight = Seconds.of(state.flightTimeSec() + kTOFChange.get());
            double tof = timeOfFlight.in(Seconds);
            if (Robot.isReal()) {
                tof += kShootDelay.getAsDouble();
            }
            effectiveGoalPose = goalPose.exp(shotMovement.toTwist2d(tof));
            distanceToGoal = PhotonUtils.getDistanceToPose(shooterPose, effectiveGoalPose);
            state = ShooterInterpolator.interpolate(
                table, distanceToGoal);
        }

        driveData.getField().getObject("SOTM Goal").setPose(effectiveGoalPose);
        
        double rpm = state.flywheelRPM() + kRPMChange.get();
        flywheel.setRealTarget(rpm);

        if(!(flywheel.atRealTarget(100)) && rpm >= flywheel.getVelocity()){
            rpm += Math.min(kFlywheelSetpointOffset.get(), 2*(rpm - flywheel.getVelocity()));
        }


        flywheel.setVelocity(rpm);
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

    public static ChassisSpeeds predictSpeeds(
        ChassisSpeeds velocity,
        ChassisAccelerations accel,
        double t
    ){
        return new ChassisSpeeds(
            velocity.vxMetersPerSecond + accel.getX()*t,
            velocity.vyMetersPerSecond + accel.getY()*t,
            velocity.omegaRadiansPerSecond + accel.getTheta()*t
        );
    }
}
