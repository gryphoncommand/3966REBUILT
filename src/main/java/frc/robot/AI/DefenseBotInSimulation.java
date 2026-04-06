package frc.robot.AI;

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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.littletonUtils.HubShiftUtil;
import frc.robot.Constants.AlignmentConstants;
import frc.robot.Constants.AutoConstants;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.drivesims.COTS;
import swervelib.simulation.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import swervelib.simulation.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import swervelib.simulation.ironmaple.simulation.motorsims.SimulatedBattery;
import swervelib.simulation.ironmaple.simulation.motorsims.SimulatedMotorController;

public class DefenseBotInSimulation extends SubsystemBase {
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

    // One active PathfindingCommand per mode.
    private Command defensePathCommand = null;

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
                COTS.WHEELS.BLUE_NITRILE_TREAD.cof * 0.9, 3));

    private final SwerveDriveKinematics kinematics =
        new SwerveDriveKinematics(enemyConfig.moduleTranslations);

    public DefenseBotInSimulation(int id, Supplier<Pose2d> goalPose, Alliance alliance) {
        this.id = id;
        this.goalPose = goalPose;
        this.alliance = alliance;

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
            pinTimer = 0.0;
            backingOff = false;
            lastSeparationTime.stop();
            lastSeparationTime.reset();
        }
        wasDefending = defending;

        runDefenseMode(currentPose);
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
    private void cancelDefensePathCommand() {
        if (defensePathCommand != null && defensePathCommand.isScheduled())
            defensePathCommand.cancel();
        defensePathCommand = null;
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