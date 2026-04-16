package frc.robot.AI;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.drivesims.COTS;
import swervelib.simulation.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import swervelib.simulation.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import swervelib.simulation.ironmaple.simulation.motorsims.SimulatedBattery;
import swervelib.simulation.ironmaple.simulation.motorsims.SimulatedMotorController;

public class StationaryBotInSimulation extends SubsystemBase {
    // =========================================
    // Shared hardware / simulation state
    // =========================================
    private final SwerveDriveSimulation driveSimulation;
    private final SimulatedMotorController.GenericMotorController[] driveControllers;
    private final SimulatedMotorController.GenericMotorController[] steerControllers;
    private final int id;

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
                COTS.WHEELS.BLUE_NITRILE_TREAD.cof, 3));

    public StationaryBotInSimulation(int id) {
        this.id = id;

        SimulatedBattery.disableBatterySim();
        driveSimulation = new SwerveDriveSimulation(enemyConfig, new Pose2d(7.75, 5.5, new Rotation2d()));
        driveSimulation.setSimulationWorldPose(new Pose2d(7.75, 5.5, new Rotation2d()));
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
        Logger.recordOutput("Drive/AI/" + id + "/Pose", currentPose);
    }

    public Pose2d getCurrentPose(){
        return driveSimulation.getSimulatedDriveTrainPose();
    }
}