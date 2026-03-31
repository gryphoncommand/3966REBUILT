package frc.robot.AI;

import static edu.wpi.first.units.Units.*;

import java.util.Random;
import java.util.Set;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.*;

import frc.FuelSim;
import frc.littletonUtils.FieldConstants;
import frc.littletonUtils.HubShiftUtil;
import frc.robot.Constants.AlignmentConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.VisionConstants;
import swervelib.simulation.ironmaple.simulation.*;
import swervelib.simulation.ironmaple.simulation.drivesims.*;
import swervelib.simulation.ironmaple.simulation.drivesims.configs.*;
import swervelib.simulation.ironmaple.simulation.motorsims.*;

public class OffensiveBotInSim extends SubsystemBase {

    private final int id;
    private final Alliance ally;
    private boolean shooting = true;
    private boolean sotm = false;
    // =========================
    // Phase enum
    // =========================
    private enum Phase { SEEK_FUEL, DRIVE_TO_GOAL, PASS, SHOOT, PASS_FULL }

    // =========================
    // Constants
    // =========================
    private static Translation2d STAGING_POINT_INIT = new Translation2d(14.5, 4.10);
    private static Translation2d STAGING_POINT = new Translation2d(14.5, 4.10);
    private static Pose2d PASS_ZONE = new Pose2d(2.412, 5.607, new Rotation2d());

    private static final double COLLECT_RADIUS = 1.0;
    private static final int MAX_CARRY = 40;
    private static final double ARRIVAL_THRESHOLD = 0.3;
    private static final double bps = 9;

    private Random passRandomizer = new Random();

    private static final Timer shotTimer = new Timer();

    // =========================
    // State
    // =========================
    private Phase phase = Phase.SEEK_FUEL;
    private int fuelsCarried = 0;
    private Translation2d cachedFuelTarget = null;

    private final SwerveDriveSimulation driveSimulation;
    private final SimulatedMotorController.GenericMotorController[] driveControllers;
    private final SimulatedMotorController.GenericMotorController[] steerControllers;

    private Command activeCommand = null;
    private Pose2d activeTarget = null;

    private final PPHolonomicDriveController pathController =
            new PPHolonomicDriveController(
                    new PIDConstants(5.0, 0.0),
                    new PIDConstants(5.0, 0.0)
            );

    @SuppressWarnings("unchecked")
    private final DriveTrainSimulationConfig config =
            new DriveTrainSimulationConfig(
                    Pounds.of(80),
                    Meters.of(0.76),
                    Meters.of(0.76),
                    Meters.of(0.52),
                    Meters.of(0.52),
                    COTS.ofGenericGyro(),
                    COTS.ofMAXSwerve(
                            DCMotor.getKrakenX60(1),
                            DCMotor.getNeo550(1),
                            COTS.WHEELS.BLUE_NITRILE_TREAD.cof * 0.8,
                            3
                    )
            );

    private final SwerveDriveKinematics kinematics =
            new SwerveDriveKinematics(config.moduleTranslations);

    // =========================
    // Constructor
    // =========================
    public OffensiveBotInSim(int id, Alliance ally, boolean sotm) {
        this.id = id;
        this.ally = ally;
        this.sotm = sotm;
        SimulatedBattery.disableBatterySim();

        driveSimulation = new SwerveDriveSimulation(
                config,
                new Pose2d(5, 2, new Rotation2d())
        );

        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);

        driveControllers = new SimulatedMotorController.GenericMotorController[4];
        steerControllers = new SimulatedMotorController.GenericMotorController[4];

        driveSimulation.setSimulationWorldPose(ROBOT_QUEENING_POSITIONS[id]);

        for (int i = 0; i < 4; i++) {
            driveControllers[i] = driveSimulation.getModules()[i].useGenericMotorControllerForDrive();
            steerControllers[i] = driveSimulation.getModules()[i].useGenericControllerForSteer();
        }

        if (ally.equals(Alliance.Red)){
            PASS_ZONE = new Pose2d(VisionConstants.kTagLayout.getFieldLength() - PASS_ZONE.getX(), VisionConstants.kTagLayout.getFieldWidth() - PASS_ZONE.getY(), new Rotation2d());
        }
        if (ally.equals(Alliance.Blue)){
            STAGING_POINT = new Translation2d(VisionConstants.kTagLayout.getFieldWidth() - STAGING_POINT.getY(), VisionConstants.kTagLayout.getFieldLength() - STAGING_POINT.getX());
        }

        shotTimer.start();
        Logger.recordOutput("Drive/AI Status", "Created AI " + id);
    }

    // =========================
    // Periodic
    // =========================
    @Override
    public void periodic() {
        Pose2d pose = driveSimulation.getSimulatedDriveTrainPose();
        Logger.recordOutput("Drive/AI/" + id + "/Pose", pose);
        Logger.recordOutput("Drive/AI/" + id + "/Passing Pose", PASS_ZONE);
        Logger.recordOutput("Drive/AI/" + id + "/Mode", phase);
        Logger.recordOutput("Drive/AI/" + id + "/Fuel Held", fuelsCarried);

        if (DriverStation.isDisabled()){
            return;
        }

        if (DriverStation.getAlliance() != null && DriverStation.getAlliance().isPresent() &&
            ally.equals(DriverStation.getAlliance().get())) {
            shooting = HubShiftUtil.getShiftedShiftInfo().active() || HubShiftUtil.getShiftedShiftInfo().remainingTime() < 3;
        } else {
            shooting = HubShiftUtil.isOpposingHubActive() || HubShiftUtil.getShiftedShiftInfo().remainingTime() < 3;
        }

        switch (phase) {
            case SEEK_FUEL -> runSeekFuel(pose);
            case DRIVE_TO_GOAL -> runDriveToGoal(pose);
            case PASS -> runSeekFuelNeutral(pose);
            case PASS_FULL -> runPass(pose);
            case SHOOT -> runShoot(pose);
        }
    }

    private void runSeekFuelNeutral(Pose2d pose) {
        if (shooting){
            phase = Phase.SEEK_FUEL;
            return;
        }
        if (fuelsCarried >= MAX_CARRY){
            phase = Phase.PASS_FULL;
        }
        if (fuelsCarried > 0 && shotTimer.get() > (1 / bps)) {
            FuelSim.getInstance().spawnFuelIfAvailable(
                new Translation3d(PASS_ZONE.getX(), PASS_ZONE.getY(), 1),
                new Translation3d(passRandomizer.nextDouble()-0.5, passRandomizer.nextDouble()-0.5, passRandomizer.nextDouble()-1)
            );
            shotTimer.restart();
            fuelsCarried--;
        }

        Set<Translation2d> fuels = FuelSim.getInstance().getFuels();
        Translation2d robot = pose.getTranslation();

        Translation2d nearest = null;
        double best = Double.MAX_VALUE;

        for (Translation2d f : fuels) {
            if (FieldConstants.isInTowerZone(f)) continue;
            if (f.getX() < 4.3 && ally.equals(Alliance.Blue) || f.getX() > 12.2 && ally.equals(Alliance.Red)) continue; // neutral zone only
            double d = robot.getDistance(f);
            if (d < best) {
                best = d;
                nearest = f;
            }
        }

        if (nearest == null) {
            if (fuelsCarried > 0) phase = Phase.DRIVE_TO_GOAL;
            return;
        }

        if (best < COLLECT_RADIUS) {
            if (FuelSim.getInstance().collectFuelAt(nearest, COLLECT_RADIUS)) {
                fuelsCarried++;
            }
            cachedFuelTarget = null;
            cancelCommand();
            return;
        }

        boolean changed = cachedFuelTarget == null || cachedFuelTarget.getDistance(nearest) > 0.5;
        if (changed) {
            cancelCommand();
            cachedFuelTarget = nearest;
        }

        if (activeCommand == null || !activeCommand.isScheduled()) {
            schedule(new Pose2d(nearest, pose.getRotation()));
        }
    }

    private void runSeekFuel(Pose2d pose) {
        if (!shooting){
            phase = Phase.PASS;
            return;
        }

        Set<Translation2d> fuels = FuelSim.getInstance().getFuels();
        Translation2d robot = pose.getTranslation();

        Translation2d nearestPriority = null;
        double bestPriority = Double.MAX_VALUE;

        Translation2d nearestFallback = null;
        double bestFallback = Double.MAX_VALUE;

        for (Translation2d f : fuels) {
            if (FieldConstants.isInTowerZone(f)) continue;
            double d = robot.getDistance(f);
            if (d <= 1.1) continue;

            boolean onPreferredSide =
                (ally.equals(Alliance.Red) && f.getX() > 12.2) ||
                (ally.equals(Alliance.Blue) && f.getX() < 3.3);

            if (sotm && onPreferredSide) {
                if (d < bestPriority) {
                    bestPriority = d;
                    nearestPriority = f;
                }
            } else {
                if (d < bestFallback) {
                    bestFallback = d;
                    nearestFallback = f;
                }
            }
        }

        Translation2d nearest = (nearestPriority != null) ? nearestPriority : nearestFallback;

        Logger.recordOutput("Drive/AI/" + id + "/Goal Fuel", new Pose2d(nearest, new Rotation2d()));

        if(sotm){
            if((ally.equals(Alliance.Red) && pose.getX() > 13.2) ||
               (ally.equals(Alliance.Blue) && pose.getX() < 3.3)) {
                if (fuelsCarried > 0 && shotTimer.get() > (1 / bps)) {
                    FuelSim.getInstance().shootFuelIntoRedHub();
                    shotTimer.restart();
                    fuelsCarried--;
                }
                if (nearest != null && ((nearest.getX() > 4.3 && nearest.getX() < 12.2) && fuelsCarried > 0)){
                    return;
                }
                if (fuelsCarried >= MAX_CARRY){
                    cancelCommand();
                    phase = Phase.SHOOT;
                    return;
                }
            }
        }
            

        if (fuelsCarried >= MAX_CARRY) {
            cancelCommand();
            phase = Phase.DRIVE_TO_GOAL;
            return;
        }

        if (nearest == null) {
            cancelCommand();
            phase = Phase.DRIVE_TO_GOAL;
            return;
        }

        if (FuelSim.getInstance().collectFuelAt(pose.getTranslation(), COLLECT_RADIUS)) {
            fuelsCarried++;
            cachedFuelTarget = null;
            cancelCommand();
        }
    

        boolean changed = cachedFuelTarget == null || cachedFuelTarget.getDistance(nearest) > 0.5;
        if (changed) {
            cancelCommand();
            cachedFuelTarget = nearest;
        }

        if (activeCommand == null || !activeCommand.isScheduled()) {
            schedule(new Pose2d(nearest, pose.getRotation()));
        }
    }

    private void runDriveToGoal(Pose2d pose) {
        Pose2d goal;

        if (ally.equals(Alliance.Blue)){
            STAGING_POINT = new Translation2d(VisionConstants.kTagLayout.getFieldWidth() - STAGING_POINT_INIT.getY(), VisionConstants.kTagLayout.getFieldLength() - STAGING_POINT_INIT.getX());
        } else {
            STAGING_POINT = new Translation2d(STAGING_POINT_INIT.getX(), STAGING_POINT_INIT.getY());
        }

        if (shooting) {
            // Head to staging point to score
            goal = new Pose2d(STAGING_POINT, new Rotation2d(ally.equals(Alliance.Red) ? Math.PI : 0));
            if (
                (ally.equals(Alliance.Red) && pose.getX() > 13.2 && pose.getX() < 14.5) ||
                (ally.equals(Alliance.Blue) && pose.getX() < 3.3 && pose.getX() > 2.0) &&
                (pose.getY() > 2.5 && pose.getY() < 5.25)
                ) {
                STAGING_POINT = pose.getTranslation();
                Pose2d hubPose = ally.equals(Alliance.Red) ? AlignmentConstants.RedHubPose : AlignmentConstants.BlueHubPose;
                Translation2d toHub = hubPose.getTranslation().minus(pose.getTranslation());
                Rotation2d rot = new Rotation2d(toHub.getX(), toHub.getY());
                goal = new Pose2d(pose.getX(), pose.getY(), rot);
            }
        } else {
            // Pass to alliance zone if shooting is false
            goal = pose;
        }

        double dist = pose.getTranslation().getDistance(goal.getTranslation());
        if (dist < ARRIVAL_THRESHOLD) {
            cancelCommand();
            phase = shooting ? (sotm ? Phase.SEEK_FUEL : Phase.SHOOT) : Phase.PASS;
            return;
        }

        schedule(goal);
    }

    // =========================
    // PASS
    // =========================
    private void runPass(Pose2d pose) {
        // Only pass if not shooting
        if (fuelsCarried > 0 && shotTimer.get() > (1 / bps)) {
            FuelSim.getInstance().spawnFuelIfAvailable(
                new Translation3d(PASS_ZONE.getX(), PASS_ZONE.getY(), 1),
                new Translation3d(passRandomizer.nextDouble()-0.5, passRandomizer.nextDouble()-0.5, passRandomizer.nextDouble()-1)
            );
            shotTimer.restart();
            fuelsCarried--;
        }

        if (fuelsCarried <= MAX_CARRY - 5 || shooting) {
            cachedFuelTarget = null;
            phase = Phase.PASS;
        }
    }

    // =========================
    // SHOOT
    // =========================
    private void runShoot(Pose2d pose) {
        if (!sotm){
            double distToHub = pose.getTranslation().getDistance(STAGING_POINT);
            if (distToHub > 0.5) {
                phase = Phase.DRIVE_TO_GOAL;
                return;
            }
        } else {
            if ((ally.equals(Alliance.Red) && pose.getX() > 13.2) || 
            (ally.equals(Alliance.Blue) && pose.getX() < 3.3)){
                phase = Phase.SEEK_FUEL;
                return;
            }
        }
        

        if (fuelsCarried > 0 && shotTimer.get() > (1 / bps)) {
            FuelSim.getInstance().shootFuelIntoRedHub();
            shotTimer.restart();
            fuelsCarried--;
        }

        if (sotm && fuelsCarried <= MAX_CARRY - 3){
            cachedFuelTarget = null;
            phase = Phase.SEEK_FUEL;
        }

        if (fuelsCarried == 0) {
            cachedFuelTarget = null;
            phase = Phase.SEEK_FUEL;
        }
    }

    // =========================
    // PATH SCHEDULER
    // =========================
    private void schedule(Pose2d target) {
        if (activeCommand != null && activeCommand.isScheduled() &&
            activeTarget != null &&
            activeTarget.getTranslation().getDistance(target.getTranslation()) < 0.1) {
            return;
        }

        cancelCommand();
        activeTarget = target;
        try {
            activeCommand = new PathfindingCommand(
                    target,
                    AutoConstants.defenseConstraints,
                    driveSimulation::getSimulatedDriveTrainPose,
                    driveSimulation::getDriveTrainSimulatedChassisSpeedsRobotRelative,
                    (speeds, ff) -> setSpeeds(speeds),
                    pathController,
                    RobotConfig.fromGUISettings(),
                    this
            );
            CommandScheduler.getInstance().schedule(activeCommand);
        } catch (Exception e){
            e.printStackTrace();
        }

    }

    private void cancelCommand() {
        if (activeCommand != null && activeCommand.isScheduled()) activeCommand.cancel();
        activeCommand = null;
        activeTarget = null;
        setSpeeds(new ChassisSpeeds());
    }
    // =========================
    // DRIVE HELPERS
    // =========================
    private void setSpeeds(ChassisSpeeds speeds) {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, 3.5);

        double maxVoltage = 12.0;
        double freeSpeed = driveSimulation.maxLinearVelocity().in(MetersPerSecond);

        for (int i = 0; i < 4; i++) {
            driveControllers[i].requestVoltage(
                    Volts.of((states[i].speedMetersPerSecond / freeSpeed) * maxVoltage)
            );
            steerControllers[i].requestVoltage(
                    Volts.of(steerPID(states[i].angle.getRadians(),
                            driveSimulation.getModules()[i].getSteerAbsoluteFacing().getRadians()))
            );
        }
    }

    private double steerPID(double target, double current) {
        double error = MathUtil.angleModulus(target - current);
        return MathUtil.clamp(error * 10.0, -12.0, 12.0);
    }

    public Pose2d getCurrentPose() {
        return driveSimulation.getSimulatedDriveTrainPose();
    }

    public static final Pose2d[] ROBOT_QUEENING_POSITIONS = new Pose2d[] {
        new Pose2d(3,  2, new Rotation2d()),
        new Pose2d(5,  2, new Rotation2d()),
        new Pose2d(6,  2, new Rotation2d()),
        new Pose2d(12.2, 0.6444996, new Rotation2d()),
        new Pose2d(10, 4, new Rotation2d())
    };
}