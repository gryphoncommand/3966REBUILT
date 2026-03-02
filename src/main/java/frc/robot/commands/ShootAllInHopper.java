package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.filter.Debouncer;
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
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.Indexing.FeedShooterFactory;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Flywheel.FlywheelIO;
import frc.robot.subsystems.Flywheel.FlywheelSimTalonFX;
import frc.robot.subsystems.Hood.HoodIO;
import frc.robot.subsystems.Indexer.Kicker;
import frc.robot.subsystems.Indexer.PreIndexer;
import frc.robot.subsystems.Indexer.Spindexer;
import frc.robot.subsystems.Intake.IntakeRollersTalonFX;

public class ShootAllInHopper extends Command {

    private final DriveSubsystem driveData;
    private final HoodIO hood;
    private final FlywheelIO flywheel;
    private final Spindexer spindexer;
    private final boolean stopFlywheelOnEnd;
    private boolean spindexerDirection = true;
    private FeedShooterFactory passthroughFactory;
    private boolean indexingStopped = true;
    private boolean neeedAlign = true;
    private boolean reachedSetpoint;
    private boolean startedShooting;
	private final Debouncer endTrigger = new Debouncer(0.5);
    private final Timer timer = new Timer();
    private double reachedSetpointTime = 0.0;

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
    public ShootAllInHopper(DriveSubsystem driveData, HoodIO hood, FlywheelIO flywheel, IntakeRollersTalonFX rollers, Kicker kicker, PreIndexer preIndexer, Spindexer spindexer, boolean stopFlywheelOnEnd, boolean neeedAlign) {
        this.driveData = driveData;
        this.hood = hood;
        this.flywheel = flywheel;
        this.stopFlywheelOnEnd = stopFlywheelOnEnd;
        this.spindexer = spindexer;
        this.neeedAlign = neeedAlign;

        if (stopFlywheelOnEnd){
            addRequirements(flywheel);
        }

        passthroughFactory = new FeedShooterFactory(kicker, preIndexer, spindexer);
        addRequirements(kicker, preIndexer, spindexer);
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
    public ShootAllInHopper(DriveSubsystem driveData, HoodIO hood, FlywheelIO flywheel, IntakeRollersTalonFX rollers, Kicker kicker, PreIndexer preIndexer, Spindexer spindexer) {
        this(driveData, hood, flywheel, rollers, kicker, preIndexer, spindexer, false, true);
    }

    @Override
    public void initialize() {
        setName("Shoot");
        // Rollers start stopped until shooter is ready.
        passthroughFactory.stop();
        indexingStopped = true;
        reachedSetpoint = false;
        startedShooting = false;
        timer.restart();
        reachedSetpointTime = 1000;
    }

    @Override
    public void execute() {
        if (spindexer.getStatorCurrent() >= IndexerConstants.kActiveCurrentSpindexer && timer.get() > reachedSetpointTime + 1.5){
            startedShooting = true;
        }
        // Only feed when both hood and flywheel report on-target
        boolean hoodReady = hood.atTarget(5.0);
        boolean flyReady = flywheel.atTarget(750);
        boolean aligned = driveData.getAligned();
        if (!neeedAlign){
            aligned = true;
        } 

        if (flywheel.atTarget(100) && !reachedSetpoint){
            reachedSetpoint = true;
            reachedSetpointTime = timer.get();
        }

        if (!reachedSetpoint){
            return;
        }
        
        if (Robot.isSimulation()){ 
            flyReady = flywheel.atTarget(50);
            if (hoodReady && flyReady && aligned &&  spindexer.hasBalls()) {
                double kShooterEfficiency = 0.48;

                double wheelRPM = flywheel.getVelocity(); // RPM
                double wheelRadPerSec = wheelRPM * 2 * Math.PI / 60;
                double wheelRadius = Units.inchesToMeters(2);
                double ballSpeed = wheelRadPerSec * wheelRadius * kShooterEfficiency;

                Pose2d ballPose2d = driveData.getCurrentPose().transformBy(ShooterConstants.kRobotToShooter);
                Translation3d initialPosition = new Translation3d(ballPose2d.getX(), ballPose2d.getY(), Units.inchesToMeters(17.701451));
                FuelSim.getInstance().spawnFuel(initialPosition, launchVel(MetersPerSecond.of(ballSpeed), Degrees.of(90 - hood.getAngle())));

                spindexer.removeBall();
                ((FlywheelSimTalonFX)flywheel).simulateShot(ballSpeed);
            }
        }

        if (hoodReady && flyReady && indexingStopped && aligned) {
            passthroughFactory.start(spindexerDirection);
            indexingStopped = false;
        } else if (!(aligned && flyReady && hoodReady)) {
            passthroughFactory.stop();
            indexingStopped = true;
        }

        passthroughFactory.periodic();
    }

    @Override
    public boolean isFinished() {
        if (Robot.isReal()) {
            if (startedShooting){
                return endTrigger.calculate(spindexer.getStatorCurrent() < IndexerConstants.kActiveCurrentSpindexer);
            }
            return false;
		} else {
			return !spindexer.hasBalls();
		}
    }

    @Override

    public void end(boolean interrupted) {
        // Always stop feeding rollers. Optionally stop flywheel.
        passthroughFactory.stop();
        if (stopFlywheelOnEnd) {
            flywheel.setVelocity(0);
            flywheel.set(0);
        }
    }
}