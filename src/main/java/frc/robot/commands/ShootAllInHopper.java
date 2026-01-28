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
import frc.robot.subsystems.Flywheel.FlywheelIO;
import frc.robot.subsystems.Hood.HoodIO;
import frc.robot.subsystems.Intake.IntakeRollersSparkFlex;

/**
 * Shoots all balls currently recorded in the hopper (simBalls).
 * This command will keep firing until the intake/feeder reports no more balls.
 * In simulation it spawns fuel via FuelSim (and decrements the simulated ball count).
 */
public class ShootAllInHopper extends Command {

	private final DriveSubsystem driveData;
	private final HoodIO hood;
	private final FlywheelIO flywheel;
	private final IntakeRollersSparkFlex intakeRollers;

	private double lastShotTime = 0.0;
	private static final double kShotIntervalSim = 0.15; // seconds between shots

	public ShootAllInHopper(DriveSubsystem driveData, HoodIO hood, FlywheelIO flywheel, IntakeRollersSparkFlex rollers) {
		this.driveData = driveData;
		this.hood = hood;
		this.flywheel = flywheel;
		this.intakeRollers = rollers;

	}

	@Override
	public void initialize() {
		setName("Shoot All In Hopper");
	}

	@Override
	public void execute() {
		boolean hoodReady = hood.atTarget(3.0);
		boolean flyReady = flywheel.atTarget(50);

		if (Robot.isSimulation()) {
			double now = Timer.getFPGATimestamp();

			if (hoodReady && flyReady && now - lastShotTime > kShotIntervalSim && intakeRollers.hasBalls()) {
				double kShooterEfficiency = 0.7;

				double wheelRPM = flywheel.getVelocity(); // RPM
				double wheelRadPerSec = wheelRPM * 2 * Math.PI / 60.0;
				double wheelRadius = Units.inchesToMeters(2.0);
				double ballSpeed = wheelRadPerSec * wheelRadius * kShooterEfficiency;

				Pose2d ballPose2d = driveData.getCurrentPose().transformBy(ShooterConstants.kRobotToShooter);
				Translation3d initialPosition =
					new Translation3d(ballPose2d.getX(), ballPose2d.getY(), Units.inchesToMeters(17.701451));

				FuelSim.getInstance().spawnFuel(initialPosition, launchVel(MetersPerSecond.of(ballSpeed), Degrees.of(90 - hood.getAngle())));

				lastShotTime = now;
				intakeRollers.removeBall();
			} else {
                if (hoodReady && flyReady){
                    // I dunno what i was thinking here lol
                    //passthrough.run();
                }
            }
		}
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

	@Override
	public boolean isFinished() {
		// Finish when there are no more balls recorded in the intake/feeder.
		return !intakeRollers.hasBalls();
	}

	@Override
	public void end(boolean interrupted) {

    }
}
