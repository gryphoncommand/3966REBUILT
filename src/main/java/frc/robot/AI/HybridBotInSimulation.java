package frc.robot.AI;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Volts;

import java.util.Set;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.FuelSim;
import frc.littletonUtils.HubShiftUtil;
import frc.robot.Constants.AlignmentConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.VisionConstants;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.drivesims.COTS;
import swervelib.simulation.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import swervelib.simulation.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import swervelib.simulation.ironmaple.simulation.motorsims.SimulatedBattery;
import swervelib.simulation.ironmaple.simulation.motorsims.SimulatedMotorController;

public class HybridBotInSimulation extends SubsystemBase {

    // =========================================
    // Idle mode: seek fuel -> drive to shoot -> shoot
    // =========================================

    private enum IdlePhase { SEEK_FUEL, DRIVE_TO_SHOOT, SHOOT }

    /**
     * Shooting position on the red side (x > 12.2). Rotation is computed at runtime
     * so the bot faces AlignmentConstants.RedHubPose from here.
     */
    private static Translation2d STAGING_POINT = new Translation2d(14.5, 4.10);

    /** Waiting position when there is no fuel anywhere and nothing to shoot. */
    private static final Pose2d FIELD_CENTER = new Pose2d(8.27, 4.10, new Rotation2d());

    /** How close (m) the bot must be to a fuel to collect it. */
    private static final double COLLECT_RADIUS = 1.0;

    /** How many fuels to carry before heading to shoot. */
    private static final int MAX_CARRY = 20;

    private static final Timer shotTimer = new Timer();

    private static final double bps = 3;

    /**
     * If the cached fuel target drifts more than this (m) the approach path is
     * regenerated toward the new position.
     */
    private static final double FUEL_REGEN_THRESHOLD = 0.5;

    /** How close (m) the bot must be to STAGING_POINT before shooting. */
    private static final double SHOOT_ARRIVAL_THRESHOLD = 0.3;

    private IdlePhase idlePhase = IdlePhase.SEEK_FUEL;
    private int fuelsCarried = 0;
    private Translation2d cachedFuelTarget = null;

    // =========================================
    // Shared hardware / simulation state
    // =========================================
    private final SwerveDriveSimulation driveSimulation;
    private final SimulatedMotorController.GenericMotorController[] driveControllers;
    private final SimulatedMotorController.GenericMotorController[] steerControllers;
    private final Supplier<Pose2d> goalPose;
    private final int id;
    private final Alliance alliance;
    private Pose2d targetPose;

    // ---- Defense state ----
    private double pinTimer = 0.0;
    private boolean backingOff = false;
    private Timer lastSeparationTime = new Timer();

    private final double pinDistance = 0.9;
    private final double maxPinDuration = 5.0;
    private final double backoffSpeed = 3.0;
    private final double periodicDt = 0.02;

    private boolean wasDefending = false;

    // One active PathfindingCommand per mode (never both scheduled at once).
    private Command defensePathCommand = null;
    private Command idleCommand = null;
    private Pose2d idleCommandTarget = null;

    private final PPHolonomicDriveController pathController =
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0),
            new PIDConstants(5.0, 0.0)
        );

    @SuppressWarnings("unchecked")
    private final DriveTrainSimulationConfig enemyConfig =
        new DriveTrainSimulationConfig(
            Pounds.of(80),
            Meters.of(0.76),
            Meters.of(0.76),
            Meters.of(0.52),
            Meters.of(0.52),
            COTS.ofGenericGyro(),
            COTS.ofMAXSwerve(DCMotor.getKrakenX60(1), DCMotor.getNeo550(1),
                COTS.WHEELS.BLUE_NITRILE_TREAD.cof * 0.8, 3));

    private final SwerveDriveKinematics kinematics =
        new SwerveDriveKinematics(enemyConfig.moduleTranslations);

    public HybridBotInSimulation(int id, Supplier<Pose2d> goalPose, Alliance alliance) {
        this.id = id;
        this.goalPose = goalPose;
        this.alliance = alliance;

        if (alliance.equals(Alliance.Blue)){
            STAGING_POINT = new Translation2d(VisionConstants.kTagLayout.getFieldWidth() - STAGING_POINT.getY(), VisionConstants.kTagLayout.getFieldLength() - STAGING_POINT.getX());
        }

        SimulatedBattery.disableBatterySim();
        driveSimulation = new SwerveDriveSimulation(enemyConfig, ROBOT_QUEENING_POSITIONS[id]);
        driveSimulation.setSimulationWorldPose(ROBOT_QUEENING_POSITIONS[id]);
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);

        driveControllers = new SimulatedMotorController.GenericMotorController[4];
        steerControllers = new SimulatedMotorController.GenericMotorController[4];
        for (int i = 0; i < 4; i++) {
            driveControllers[i] = driveSimulation.getModules()[i].useGenericMotorControllerForDrive();
            steerControllers[i] = driveSimulation.getModules()[i].useGenericControllerForSteer();
        }

        Logger.recordOutput("Drive/AI Status", "Created AI " + id);
        shotTimer.start();

        // FuelSim.getInstance().registerRobot(
        //     0.76, // from left to right
        //     0.76, // from front to back
        //     Units.inchesToMeters(6), // from floor to top of bumpers
        //     driveSimulation::getSimulatedDriveTrainPose, // Supplier<Pose2d> of robot pose
        //     driveSimulation::getDriveTrainSimulatedChassisSpeedsFieldRelative // Supplier<ChassisSpeeds> of field-centric chassis speeds
        // );
    }

    @Override
    public void periodic() {
        Pose2d currentPose = driveSimulation.getSimulatedDriveTrainPose();
        Logger.recordOutput("Drive/AI/" + id + "/Pose", currentPose);
        boolean defending = false;

        if (DriverStation.getAlliance().isPresent() && alliance.equals(DriverStation.getAlliance().get())){
            defending = !HubShiftUtil.getShiftedShiftInfo().active();
        } else {
            defending = !HubShiftUtil.isOpposingHubActive();
        }
        Logger.recordOutput("Drive/AI/" + id + "/Defending", defending);

        if (DriverStation.isDisabled()){
            setSpeeds(new ChassisSpeeds());
        }

        // Clean up opposite-mode state on any transition
        if (defending != wasDefending) {
            cancelDefensePathCommand();
            cancelIdleCommand();
            pinTimer = 0.0;
            backingOff = false;
            lastSeparationTime.stop();
            lastSeparationTime.reset();
            idlePhase = IdlePhase.SEEK_FUEL;
            fuelsCarried = 0;
            cachedFuelTarget = null;
        }
        wasDefending = defending;

        if (defending) {
            runDefenseMode(currentPose);
        } else {
            runIdleMode(currentPose);
        }
    }

    // =========================================
    // Defense mode (original behaviour)
    // =========================================
    private void runDefenseMode(Pose2d currentPose) {
        Pose2d playerPose = goalPose.get();
        double distance = currentPose.getTranslation().getDistance(playerPose.getTranslation());

        Logger.recordOutput("Drive/AI/" + id + "/Dist", distance);

        if (distance < pinDistance) {
            pinTimer += periodicDt;
            if (lastSeparationTime.isRunning()) lastSeparationTime.restart();
        } else {
            pinTimer = 0.0;
        }

        boolean approachingPinLimit = pinTimer >= maxPinDuration - 0.5;

        if (distance < 2 || backingOff) {
            cancelDefensePathCommand();

            if (approachingPinLimit || backingOff) {
                driveBackOff(currentPose, playerPose,
                    driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative());
                backingOff = true;
                if (distance > 2 || lastSeparationTime.get() > 4.0) {
                    backingOff = false;
                    lastSeparationTime.stop();
                    lastSeparationTime.reset();
                    pinTimer = 0.0;
                }
                Logger.recordOutput("Drive/AI/" + id + "/Mode", "BACKING OFF");
            } else {
                lastSeparationTime.stop();
                lastSeparationTime.reset();
                backingOff = false;
                driveAggressive(playerPose, currentPose);
                Logger.recordOutput("Drive/AI/" + id + "/Mode", "AGGRESSIVE");
            }
        } else {
            backingOff = false;
            pinTimer = 0;
            Logger.recordOutput("Drive/AI/" + id + "/Mode", "PATHPLANNER");

            if (defensePathCommand != null && defensePathCommand.isScheduled()) {
                double endDist = targetPose.getTranslation().getDistance(playerPose.getTranslation());
                if (endDist > 1.0) cancelDefensePathCommand();
            }

            if (defensePathCommand == null || !defensePathCommand.isScheduled()) {
                try {
                    targetPose = playerPose;
                    defensePathCommand = new PathfindingCommand(
                        playerPose,
                        AutoConstants.defenseConstraints,
                        driveSimulation::getSimulatedDriveTrainPose,
                        driveSimulation::getDriveTrainSimulatedChassisSpeedsRobotRelative,
                        (speeds, feedforwards) -> setSpeeds(speeds),
                        pathController,
                        RobotConfig.fromGUISettings(),
                        this
                    ).andThen(new InstantCommand(()->setSpeeds(new ChassisSpeeds()), this));
                    CommandScheduler.getInstance().schedule(defensePathCommand);
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
        }
    }

    // =========================================
    // Idle mode dispatcher
    // =========================================
    private void runIdleMode(Pose2d currentPose) {
        Logger.recordOutput("Drive/AI/" + id + "/FuelsCarried", fuelsCarried);
        switch (idlePhase) {
            case SEEK_FUEL      -> runSeekFuel(currentPose);
            case DRIVE_TO_SHOOT -> runDriveToShoot(currentPose);
            case SHOOT          -> runShoot(currentPose);
        }
    }

    // ---- Phase 1: seek and collect fuel ----
    private void runSeekFuel(Pose2d currentPose) {
        if (fuelsCarried >= MAX_CARRY) {
            cancelIdleCommand();
            idlePhase = IdlePhase.DRIVE_TO_SHOOT;
            return;
        }

        Set<Translation2d> fieldFuels = FuelSim.getInstance().getFuels();
        Translation2d robotPos = currentPose.getTranslation();

        // Find nearest active fuel
        Translation2d nearest = null;
        double nearestDist = Double.MAX_VALUE;
        for (Translation2d fuel : fieldFuels) {
            double dist = robotPos.getDistance(fuel);
            if (dist < nearestDist) {
                nearestDist = dist;
                nearest = fuel;
            }
        }

        // No fuel on the field at all
        if (nearest == null) {
            if (fuelsCarried > 0) {
                cancelIdleCommand();
                idlePhase = IdlePhase.DRIVE_TO_SHOOT;
            } else {
                Logger.recordOutput("Drive/AI/" + id + "/Mode", "WAITING (no fuel)");
                scheduleIdleCommandIfNeeded(FIELD_CENTER);
            }
            return;
        }

        // Within pickup range — grab it
        if (nearestDist < COLLECT_RADIUS) {
            if (FuelSim.getInstance().collectFuelAt(nearest, COLLECT_RADIUS)) {
                fuelsCarried++;
            }
            cancelIdleCommand();
            cachedFuelTarget = null;
            Logger.recordOutput("Drive/AI/" + id + "/Mode", "COLLECTED (carrying " + fuelsCarried + ")");
            return;
        }

        // Regenerate path when the fuel has rolled away from our cached target
        boolean targetChanged = (cachedFuelTarget == null)
            || (cachedFuelTarget.getDistance(nearest) > FUEL_REGEN_THRESHOLD);
        if (targetChanged) {
            cancelIdleCommand();
            cachedFuelTarget = nearest;
        }

        // Approach the fuel
        if (idleCommand == null || !idleCommand.isScheduled()) {
            cachedFuelTarget = nearest;
            scheduleIdleCommandIfNeeded(new Pose2d(nearest, currentPose.getRotation()));
        }

        Logger.recordOutput("Drive/AI/" + id + "/Mode", "SEEK_FUEL (carrying " + fuelsCarried + ")");
        Logger.recordOutput("Drive/AI/" + id + "/Dist", nearestDist);
    }

    // ---- Phase 2: drive to the shooting position ----
    private Pose2d runDriveToShoot(Pose2d currentPose) {
        // Compute facing angle dynamically toward the red hub
        Translation2d toHub = AlignmentConstants.RedHubPose.getTranslation().minus(STAGING_POINT);
        Rotation2d facingHub = new Rotation2d(toHub.getX(), toHub.getY());
        Pose2d shootPose = new Pose2d(STAGING_POINT, facingHub);

        double distToShoot = currentPose.getTranslation().getDistance(STAGING_POINT);
        Logger.recordOutput("Drive/AI/" + id + "/Mode", "DRIVE_TO_SHOOT");
        Logger.recordOutput("Drive/AI/" + id + "/Dist", distToShoot);

        if (distToShoot < SHOOT_ARRIVAL_THRESHOLD && currentPose.getX() > 13) {
            cancelIdleCommand();
            
            idlePhase = IdlePhase.SHOOT;
            return shootPose;
        }

        scheduleIdleCommandIfNeeded(shootPose);
        return shootPose;
    }

    // ---- Phase 3: score all carried fuels into the red hub ----
    private void runShoot(Pose2d currentPose) {
        Logger.recordOutput("Drive/AI/" + id + "/Mode", "SHOOTING (" + fuelsCarried + ")");
        if (currentPose.getTranslation().getDistance(STAGING_POINT) > 0.5){
            idlePhase = IdlePhase.DRIVE_TO_SHOOT;
            return;
        } 
        if (shotTimer.get() > (1/bps)){
            FuelSim.getInstance().shootFuelIntoRedHub();
            shotTimer.restart();
            fuelsCarried --;
        }
        if (fuelsCarried == 0){
            cachedFuelTarget = null;
            idlePhase = IdlePhase.SEEK_FUEL;
        }
    }

    // =========================================
    // Path-scheduling helper
    // =========================================

    /**
     * Schedules idleCommand toward {@code dest} only if the running command isn't
     * already heading to within 0.1 m of that destination.
     */
    private void scheduleIdleCommandIfNeeded(Pose2d dest) {
        if (idleCommand != null && idleCommand.isScheduled()
                && idleCommandTarget != null
                && idleCommandTarget.getTranslation().getDistance(dest.getTranslation()) < 0.1) {
            return;
        }
        cancelIdleCommand();
        idleCommandTarget = dest;
        try {
            idleCommand = new PathfindingCommand(
                dest,
                AutoConstants.defenseConstraints,
                driveSimulation::getSimulatedDriveTrainPose,
                driveSimulation::getDriveTrainSimulatedChassisSpeedsRobotRelative,
                (speeds, feedforwards) -> setSpeeds(speeds),
                pathController,
                RobotConfig.fromGUISettings(),
                this
            ).andThen(new InstantCommand(()->setSpeeds(new ChassisSpeeds()), this));
            CommandScheduler.getInstance().schedule(idleCommand);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    private void cancelDefensePathCommand() {
        if (defensePathCommand != null && defensePathCommand.isScheduled())
            defensePathCommand.cancel();
        defensePathCommand = null;
    }

    private void cancelIdleCommand() {
        if (idleCommand != null && idleCommand.isScheduled()) idleCommand.cancel();
        idleCommand = null;
        idleCommandTarget = null;
        setSpeeds(new ChassisSpeeds());
        driveSimulation.setRobotSpeeds(new ChassisSpeeds());
    }

    // =========================================
    // Low-level drive helpers (unchanged)
    // =========================================
    private void driveAggressive(Pose2d playerPose, Pose2d currentPose) {
        Translation2d playerPos = playerPose.getTranslation();
        Translation2d robotPos = currentPose.getTranslation();

        Translation2d toHub = AlignmentConstants.HubPose.getTranslation().minus(playerPos);
        double hubNorm = toHub.getNorm();
        if (hubNorm < 0.05) return;

        Translation2d hubDir = toHub.div(hubNorm);
        Translation2d perpLeft  = new Translation2d(-hubDir.getY(),  hubDir.getX());
        Translation2d perpRight = new Translation2d( hubDir.getY(), -hubDir.getX());

        double distLeft  = robotPos.getDistance(playerPos.plus(perpLeft));
        double distRight = robotPos.getDistance(playerPos.plus(perpRight));
        Translation2d chosenPerp = (distLeft < distRight) ? perpLeft : perpRight;

        Translation2d targetPoint = playerPos.plus(chosenPerp.times(0.05));
        Translation2d toTarget = targetPoint.minus(robotPos);
        double norm = toTarget.getNorm();

        if (norm < 0.05) {
            for (int i = 0; i < 4; i++) driveControllers[i].requestVoltage(Volts.of(0));
            return;
        }

        double maxSpeed = 3.0;
        Translation2d velocity = toTarget.div(norm).times(maxSpeed);

        setVelocity(velocity);
    }

    private void setVelocity(Translation2d velocity){
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            velocity.getX(), velocity.getY(), 0.0, driveSimulation.getSimulatedDriveTrainPose().getRotation());
        setSpeeds(speeds);
    }

    private void setSpeeds(ChassisSpeeds speeds){
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, 3.5);

        double maxVoltage = 12.0;
        double freeSpeedMPS = driveSimulation.maxLinearVelocity().in(MetersPerSecond);

        for (int i = 0; i < 4; i++) {
            driveControllers[i].requestVoltage(
                Volts.of((states[i].speedMetersPerSecond / freeSpeedMPS) * maxVoltage).times(0.95));
            steerControllers[i].requestVoltage(Volts.of(steerPID(
                states[i].angle.getRadians(),
                driveSimulation.getModules()[i].getSteerAbsoluteFacing().getRadians())));
        }
    }

    private double steerPID(double targetRad, double currentRad) {
        double error = MathUtil.angleModulus(targetRad - currentRad);
        return MathUtil.clamp(error * 10.0, -12.0, 12.0);
    }

    private void driveBackOff(Pose2d currentPose, Pose2d targetPose, ChassisSpeeds currentSpeeds) {
        Translation2d away = currentPose.getTranslation().minus(targetPose.getTranslation());
        double distance = away.getNorm();
        Translation2d velocity = distance > 0.05
            ? away.div(distance).times(backoffSpeed)
            : new Translation2d(0, 0);

        setVelocity(velocity);
    }

    public static final Pose2d[] ROBOT_QUEENING_POSITIONS = new Pose2d[] {
        new Pose2d(3,  2, new Rotation2d()),
        new Pose2d(5,  2, new Rotation2d()),
        new Pose2d(6,  2, new Rotation2d()),
        new Pose2d(12.2, 0.6444996, new Rotation2d()),
        new Pose2d(10, 4, new Rotation2d())
    };

    public Pose2d getCurrentPose(){
        return driveSimulation.getSimulatedDriveTrainPose();
    }
}