package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.FuelSim;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Passthrough;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Flywheel.FlywheelIO;
import frc.robot.subsystems.Hood.HoodIO;
import frc.robot.subsystems.Intake.IntakeRollersTalonFX;

public class Shoot extends Command {

    //TODO: Implement passthrough
    private final DriveSubsystem driveData;
    private final HoodIO hood;
    private final FlywheelIO flywheel;
    private final IntakeRollersTalonFX intakeRollerData;
    private final Passthrough passthrough;
    private final Spindexer spindexer;
    private final double feedRPM;
    private final boolean stopFlywheelOnEnd;
    private double lastShotTime = 0;

    /**
     * Shoot command: runs the flywheel (assumed already set) and only feeds balls
     * into the shooter once hood and flywheel are at their targets.
     *
     * @param hood hood subsystem (used to check atTarget)
     * @param flywheel flywheel subsystem (used to check atTarget)
     * @param rollers intake/feeder rollers used to feed fuel into the flywheel
     * @param feedRPM roller velocity to use when feeding
     * @param stopFlywheelOnEnd if true, zeroes the flywheel when the command ends
     */
    public Shoot(DriveSubsystem driveData, HoodIO hood, FlywheelIO flywheel, IntakeRollersTalonFX rollers, Passthrough passthrough, Spindexer spindexer, double feedRPM, boolean stopFlywheelOnEnd) {
        this.driveData = driveData;
        this.hood = hood;
        this.flywheel = flywheel;
        this.intakeRollerData = rollers;
        this.passthrough = passthrough;
        this.spindexer = spindexer;
        this.feedRPM = feedRPM;
        this.stopFlywheelOnEnd = stopFlywheelOnEnd;

        if (stopFlywheelOnEnd){
            addRequirements(flywheel);
        }

        addRequirements(passthrough);
    }

    private Translation3d launchVel(LinearVelocity vel, Angle angle) {
        Pose3d robot = new Pose3d(driveData.getCurrentPose());
        ChassisSpeeds fieldSpeeds = driveData.getCurrentSpeedsFieldRelative();

        double horizontalVel = Math.cos(angle.in(Radians)) * vel.in(MetersPerSecond);
        double verticalVel = Math.sin(angle.in(Radians)) * vel.in(MetersPerSecond);
        double xVel =
                horizontalVel * Math.cos(robot.getRotation().toRotation2d().getRadians() + Math.PI / 2);
        double yVel =
                horizontalVel * Math.sin(robot.getRotation().toRotation2d().getRadians() + Math.PI / 2);

        xVel += fieldSpeeds.vxMetersPerSecond;
        yVel += fieldSpeeds.vyMetersPerSecond;

        return new Translation3d(xVel, yVel, verticalVel);
    }

    /**
     * Convenience constructor that leaves the flywheel running when command ends.
     */
    public Shoot(DriveSubsystem drive, HoodIO hood, FlywheelIO flywheel, IntakeRollersTalonFX rollers, Passthrough passthrough, Spindexer spindexer, double feedRPM) {
        this(drive, hood, flywheel, rollers, passthrough, spindexer, feedRPM, false);
    }

    @Override
    public void initialize() {
        setName("Shoot");
        // Rollers start stopped until shooter is ready.
        passthrough.set(0);
    }

    @Override
    public void execute() {
        // Only feed when both hood and flywheel report on-target
        boolean hoodReady = hood.atTarget(3.0);
        boolean flyReady = flywheel.atTarget(50);
        boolean aligned = driveData.getAligned();
        
        if (Robot.isSimulation()){ 
            double now = Timer.getFPGATimestamp();

            if (hoodReady && flyReady && aligned && now - lastShotTime > 0.12 && spindexer.hasBalls()) {
                double kShooterEfficiency = 0.7;

                double wheelRPM = flywheel.getVelocity(); // RPM
                double wheelRadPerSec = wheelRPM * 2 * Math.PI / 60;
                double wheelRadius = Units.inchesToMeters(2);
                double ballSpeed = wheelRadPerSec * wheelRadius * kShooterEfficiency;

                Pose2d ballPose2d = driveData.getCurrentPose().transformBy(ShooterConstants.kRobotToShooter);
                Translation3d initialPosition = new Translation3d(ballPose2d.getX(), ballPose2d.getY(), Units.inchesToMeters(17.701451));
                FuelSim.getInstance().spawnFuel(initialPosition, launchVel(MetersPerSecond.of(ballSpeed), Degrees.of(90 - hood.getAngle())));

                lastShotTime = now;
                spindexer.removeBall();
            }
            
        }

        // if (hoodReady && flyReady) {
        //     passthrough.setVelocity(feedRPM);
        // } else {
        //     passthrough.stop();
        // }
    }

    @Override
    public boolean isFinished() {
        // This command is intended to be run while operator holds a button or scheduled/terminated by higher-level logic.
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // Always stop feeding rollers. Optionally stop flywheel.
        // passthrough.stop();
        if (stopFlywheelOnEnd) {
            flywheel.setVelocity(0);
        }
    }
}