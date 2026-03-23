package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import java.util.Random;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.FuelSim;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.Indexing.FeedShooterFactory;
import frc.robot.Robot;
import frc.robot.subsystems.SimDriveSubsystem;
import frc.robot.subsystems.Flywheel.FlywheelIO;
import frc.robot.subsystems.Flywheel.FlywheelSimTalonFX;
import frc.robot.subsystems.Indexer.Kicker;
import frc.robot.subsystems.Indexer.PreIndexer;
import frc.robot.subsystems.Indexer.Spindexer;
import frc.robot.subsystems.Intake.IntakeDeployIO;

public class ShootAllInHopper extends Command {

    private final SimDriveSubsystem driveData;
    private final FlywheelIO flywheel;
    private final Spindexer spindexer;
    private final boolean stopFlywheelOnEnd;
    private final IntakeDeployIO intake;
    private boolean spindexerDirection = false;
    private FeedShooterFactory passthroughFactory;
    private Debouncer endTrigger = new Debouncer(1.5);
    private boolean startedShooting = false;
    private boolean indexingStopped = true;
    private boolean neeedAlign = true;
    private boolean reachedSetpoint;
    private boolean agitateAngle;
    private Timer shootingTimer = new Timer();
    private Random simEnder = new Random();
    private Random simShotRandom = new Random();
    private int simEnd;
    private double[] nextShotTimeSec = new double[0];
    private double[] simShooterYOffsets = new double[0];

    /**
     * Shoot command: runs the flywheel (assumed already set) and only feeds balls
     * into the shooter once the flywheel is at its target.
     *
     * @param flywheel flywheel subsystem (used to check atTarget)
     * @param stopFlywheelOnEnd if true, zeroes the flywheel when the command ends
     */
    public ShootAllInHopper(SimDriveSubsystem driveData, FlywheelIO flywheel, Kicker kicker, PreIndexer preIndexer, Spindexer spindexer, IntakeDeployIO intake, boolean stopFlywheelOnEnd, boolean neeedAlign) {
        this.driveData = driveData;
        this.flywheel = flywheel;
        this.stopFlywheelOnEnd = stopFlywheelOnEnd;
        this.spindexer = spindexer;
        this.neeedAlign = neeedAlign;
        this.intake = intake;

        if (stopFlywheelOnEnd){
            addRequirements(flywheel);
        }

        passthroughFactory = new FeedShooterFactory(kicker, preIndexer, spindexer);
        addRequirements(kicker, preIndexer, spindexer, intake);
    }

    private Translation3d launchVel(LinearVelocity vel, Angle angle) {
        Pose3d robot = new Pose3d(driveData.getCurrentPose());
        ChassisSpeeds fieldSpeeds = driveData.getCurrentSpeedsFieldRelative();

        double horizontalVel = Math.cos(angle.in(Radians)) * vel.in(MetersPerSecond);
        double verticalVel = Math.sin(angle.in(Radians)) * vel.in(MetersPerSecond);
        double xVel =
                horizontalVel * Math.cos(robot.getRotation().toRotation2d().getRadians());
        double yVel =
                horizontalVel * Math.sin(robot.getRotation().toRotation2d().getRadians());

        xVel += fieldSpeeds.vxMetersPerSecond;
        yVel += fieldSpeeds.vyMetersPerSecond;

        return new Translation3d(xVel, yVel, verticalVel);
    }

    /**
     * Convenience constructor that leaves the flywheel running when command ends.
     */
    public ShootAllInHopper(SimDriveSubsystem driveData, FlywheelIO flywheel, Kicker kicker, PreIndexer preIndexer, Spindexer spindexer, IntakeDeployIO intake) {
        this(driveData, flywheel, kicker, preIndexer, spindexer, intake, false, true);
    }

    @Override
    public void initialize() {
        agitateAngle = false;
        setName("Shoot");
        // Rollers start stopped until shooter is ready.
        passthroughFactory.stop();
        indexingStopped = true;
        reachedSetpoint = false;
        startedShooting = false;
        if (Robot.isSimulation()){
            simEnd = simEnder.nextInt(0, 6);
            simShooterYOffsets = buildShooterYOffsets();
            nextShotTimeSec = new double[ShooterConstants.kSimShooterCount];
            double now = Timer.getFPGATimestamp();
            for (int i = 0; i < nextShotTimeSec.length; i++){
                nextShotTimeSec[i] = now + randomIntervalSec();
            }
        }
    }

    @Override
    public void execute() {
        // Only feed when flywheel reports on-target
        boolean flyReady = flywheel.atRealTarget(750);
        boolean aligned = driveData.getAligned();
        if (!neeedAlign){
            aligned = true;
        }

        if (!reachedSetpoint && flywheel.atRealTarget(50)){
            Logger.recordOutput("Shoot Report", "Started Shooting");
            reachedSetpoint = true;
            shootingTimer.restart();
        }
        
        if (!reachedSetpoint){
            return;
        }
        
        if (Robot.isSimulation()){ 
            double now = Timer.getFPGATimestamp();

            if (flyReady && aligned && spindexer.getBalls() != 0) {
                double kShooterEfficiency = 0.7;

                double wheelRPM = flywheel.getVelocity(); // RPM
                double wheelRadPerSec = wheelRPM * 2 * Math.PI / 60;
                double wheelRadius = Units.inchesToMeters(2);
                double ballSpeed = wheelRadPerSec * wheelRadius * kShooterEfficiency;

                for (int i = 0; i < nextShotTimeSec.length; i++){
                    if (spindexer.getBalls() == 0){
                        break;
                    }
                    if (now >= nextShotTimeSec[i]){
                        startedShooting = true;
                        Pose2d ballPose2d = driveData.getRealPoseSim().transformBy(
                            new Transform2d(
                                ShooterConstants.kSimShooterXOffsetMeters,
                                simShooterYOffsets[i],
                                new Rotation2d()
                            )
                        );
                        Translation3d initialPosition = new Translation3d(ballPose2d.getX(), ballPose2d.getY(), Units.inchesToMeters(17.701451));
                        boolean spawned = FuelSim.getInstance().spawnFuelIfAvailable(
                            initialPosition,
                            launchVel(MetersPerSecond.of(ballSpeed), Degrees.of(90 - ShooterConstants.kFixedHoodAngleDeg)));
                        if (spawned) {
                            nextShotTimeSec[i] = now + randomIntervalSec();
                            spindexer.removeBall();
                            ((FlywheelSimTalonFX)flywheel).simulateShot(wheelRadPerSec * wheelRadius);
                        }
                    }
                }
            }
        }

        if (flyReady && indexingStopped && aligned) {
            Logger.recordOutput("Shoot Report", "Shooting");
            startedShooting = true;
            passthroughFactory.start(spindexerDirection);
            indexingStopped = false;
        } else if (!(aligned && flyReady)) {
            intake.setPosition(IntakeConstants.kIntakeDeployAngle);
            Logger.recordOutput("Shoot Report", "Shooter Not Ready, Align " + aligned + ", Flywheel " + flyReady);
            passthroughFactory.stop();
            indexingStopped = true;
        }

        if (flyReady && aligned){
            if (intake.atTarget(0.02)){
                if (agitateAngle){
                    intake.setPosition(IntakeConstants.kIntakeAgitateAngle);
                } else {
                    intake.setPosition(IntakeConstants.kIntakeDeployAngle);
                }

                agitateAngle = !agitateAngle;
            }
        }
        passthroughFactory.periodic();
    }

    @Override
    public boolean isFinished() {
        if (Robot.isReal()) {
            if (startedShooting && shootingTimer.get() > 3){
                return endTrigger.calculate(spindexer.getStatorCurrent() < IndexerConstants.kActiveCurrentSpindexer);
            }
            return false;
		} else {
			return spindexer.getBalls() <= simEnd;
		}
    }

    @Override
    public void end(boolean interrupted) {
        // Always stop feeding rollers. Optionally stop flywheel.
        passthroughFactory.stop();
        intake.setPosition(IntakeConstants.kIntakeDeployAngle);
        if (stopFlywheelOnEnd) {
            flywheel.setVelocity(0);
            flywheel.set(0);
        }
    }

    private double randomIntervalSec(){
        double mean = ShooterConstants.kSimPerShooterMeanIntervalSec;
        double jitter = ShooterConstants.kSimShotIntervalJitterFrac;
        double min = mean * (1.0 - jitter);
        double max = mean * (1.0 + jitter);
        return min + simShotRandom.nextDouble() * (max - min);
    }

    private double[] buildShooterYOffsets(){
        int count = ShooterConstants.kSimShooterCount;
        double spacing = ShooterConstants.kSimShooterYSpacingMeters;
        double[] offsets = new double[count];
        double start = -spacing * (count - 1) / 2.0;
        for (int i = 0; i < count; i++){
            offsets[i] = start + i * spacing;
        }
        return offsets;
    }
}
