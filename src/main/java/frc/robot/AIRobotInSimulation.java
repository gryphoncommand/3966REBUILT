package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Volts;

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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;

import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.drivesims.COTS;
import swervelib.simulation.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import swervelib.simulation.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import swervelib.simulation.ironmaple.simulation.motorsims.SimulatedBattery;
import swervelib.simulation.ironmaple.simulation.motorsims.SimulatedMotorController;

public class AIRobotInSimulation extends SubsystemBase {

    private final SwerveDriveSimulation driveSimulation;
    private final SimulatedMotorController.GenericMotorController[] driveControllers;
    private final SimulatedMotorController.GenericMotorController[] steerControllers;
    private final Supplier<Pose2d> goalPose;
    private final int id;
    private Pose2d targetPose;

    private double pinTimer = 0.0;
    private boolean backingOff = false;
    private Timer lastSeparationTime = new Timer();

    private final double pinDistance = 0.9; // meters (72 inches)
    private final double maxPinDuration = 5.0; // seconds
    private final double backoffSpeed = 3.0; // m/s
    private final double periodicDt = 0.02; // simulation tick

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
        COTS.ofMAXSwerve(DCMotor.getKrakenX60(1), DCMotor.getNeo550(1), COTS.WHEELS.BLUE_NITRILE_TREAD.cof*0.8, 3));
    
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(enemyConfig.moduleTranslations);

    private Command pathCommand = null;

    public AIRobotInSimulation(int id, Supplier<Pose2d> goalPose) {
        this.id = id;
        this.goalPose = goalPose;

        SimulatedBattery.disableBatterySim();
        driveSimulation = new SwerveDriveSimulation(
            enemyConfig,
            ROBOT_QUEENING_POSITIONS[id]
        );
        driveSimulation.setSimulationWorldPose(ROBOT_QUEENING_POSITIONS[id]);
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);

        driveControllers = new SimulatedMotorController.GenericMotorController[4];
        steerControllers = new SimulatedMotorController.GenericMotorController[4];
        for (int i = 0; i < 4; i++) {
            driveControllers[i] = driveSimulation.getModules()[i].useGenericMotorControllerForDrive();
            steerControllers[i] = driveSimulation.getModules()[i].useGenericControllerForSteer();
        }

        Logger.recordOutput("Drive/AI Status", "Created AI " + id);
    }

    @Override
    public void periodic() {
        Pose2d currentPose = driveSimulation.getSimulatedDriveTrainPose();
        Pose2d playerPose = goalPose.get();
        double distance = currentPose.getTranslation()
                .getDistance(playerPose.getTranslation());

        Logger.recordOutput("Drive/AI/" + id + "/Pose", currentPose);
        Logger.recordOutput("Drive/AI/" + id + "/Dist", distance);


        // =========================================
        // Smart pinning avoidance logic
        // =========================================
        if (distance < pinDistance) {
            pinTimer += periodicDt;
            if (lastSeparationTime.isRunning()){
                lastSeparationTime.restart();
            }
        } else {
            pinTimer = 0.0; // reset if robot is far enough
        }

        boolean approachingPinLimit = pinTimer >= maxPinDuration - 0.5;

        // =========================================
        // Aggressive interception (but respecting pinning rules)
        // =========================================
        if (distance < 2 || backingOff) {
            if (pathCommand != null && pathCommand.isScheduled()) {
                pathCommand.cancel();
            }

            if (approachingPinLimit || backingOff) {
                // Back off to avoid pinning penalty
                driveBackOff(currentPose, playerPose, driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative());
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

            
        }
        // =========================================
        // Far-range: dynamic pathfinding with navgrid
        // =========================================
        else {
            backingOff = false;
            pinTimer = 0;
            Logger.recordOutput("Drive/AI/" + id + "/Mode", "PATHPLANNER");

            // Cancel old path if outdated
            if (pathCommand != null && pathCommand.isScheduled()) {
                double endDist = targetPose.getTranslation().getDistance(playerPose.getTranslation());
                if (endDist > 1.0) pathCommand.cancel();
            }

            // Generate a new dynamic path toward the player
            if (pathCommand == null || !pathCommand.isScheduled()) {
                try {
                    targetPose = playerPose;
                    pathCommand = new PathfindingCommand(
                        playerPose,
                        AutoConstants.defenseConstraints,
                        driveSimulation::getSimulatedDriveTrainPose,
                        driveSimulation::getDriveTrainSimulatedChassisSpeedsRobotRelative,
                        (speeds, feedforwards) ->
                            driveSimulation.setRobotSpeeds(
                                ChassisSpeeds.fromRobotRelativeSpeeds(
                                    speeds,
                                    driveSimulation.getSimulatedDriveTrainPose().getRotation()
                                )
                            ),
                        pathController,
                        RobotConfig.fromGUISettings(),
                        this
                    );
                    CommandScheduler.getInstance().schedule(pathCommand);
                    
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
        }
    }

    private void driveAggressive(Pose2d targetPose, Pose2d currentPose) {
        Translation2d toPlayer = targetPose.getTranslation().minus(currentPose.getTranslation());
        double norm = toPlayer.getNorm();
        if (norm < 0.05) {
            for (int i = 0; i < 4; i++)
                driveControllers[i].requestVoltage(Volts.of(0));
            return;
        }

        double maxSpeed = 3;
        Translation2d velocity = toPlayer.div(norm).times(maxSpeed);

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            velocity.getX(), velocity.getY(), 0.0,
            currentPose.getRotation()
        );

        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, maxSpeed);

        double maxVoltage = 12.0;
        double freeSpeedMPS = driveSimulation.maxLinearVelocity().in(MetersPerSecond);

        for (int i = 0; i < 4; i++) {
            double voltageOut = (states[i].speedMetersPerSecond / freeSpeedMPS) * maxVoltage;
            driveControllers[i].requestVoltage(Volts.of(voltageOut));
            steerControllers[i].requestVoltage(
                Volts.of(steerPID(
                    states[i].angle.getRadians(),
                    driveSimulation.getModules()[i].getSteerAbsoluteFacing().getRadians()
                ))
            );
        }
    }

    // Simple P controller for steer — tune kP as needed
    private double steerPID(double targetRad, double currentRad) {
        double error = MathUtil.angleModulus(targetRad - currentRad);
        return MathUtil.clamp(error * 10.0, -12.0, 12.0);
}

    private void driveBackOff(Pose2d currentPose, Pose2d targetPose, ChassisSpeeds currentSpeeds) {
            Translation2d away = currentPose.getTranslation()
                    .minus(targetPose.getTranslation());

            double distance = away.getNorm();
            Translation2d velocity = distance > 0.05
                    ? away.div(distance).times(backoffSpeed)
                    : new Translation2d(0, 0);

            ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    velocity.getX(),
                    velocity.getY(),
                    0.0,
                    currentPose.getRotation()
            );

            SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
            SwerveDriveKinematics.desaturateWheelSpeeds(states, backoffSpeed);

            double maxVoltage = 12.0;
            double freeSpeedMPS = driveSimulation.maxLinearVelocity().in(MetersPerSecond);

            for (int i = 0; i < 4; i++) {
                double voltageOut = (states[i].speedMetersPerSecond / freeSpeedMPS) * maxVoltage;
                driveControllers[i].requestVoltage(Volts.of(voltageOut));
                steerControllers[i].requestVoltage(
                    Volts.of(steerPID(
                        states[i].angle.getRadians(),
                        driveSimulation.getModules()[i].getSteerAbsoluteFacing().getRadians()
                    ))
                );
            }
        }

    public static final Pose2d[] ROBOT_QUEENING_POSITIONS = new Pose2d[] {
        new Pose2d(3, 2, new Rotation2d()),
        new Pose2d(5, 2, new Rotation2d()),
        new Pose2d(6, 2, new Rotation2d()),
        new Pose2d(8, 3, new Rotation2d()),
        new Pose2d(10, 4, new Rotation2d())
    };
}