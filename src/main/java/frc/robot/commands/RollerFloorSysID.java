package frc.robot.commands;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.Indexer.PreIndexer;

public class RollerFloorSysID {    
    SysIdRoutine routine;
    PreIndexer rollers;

    public RollerFloorSysID(PreIndexer rollers){
        routine = new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism((volts) -> rollers.setVoltage(volts.in(Volts)), 
        log -> {
                // Record a frame for the shooter motor.
                log.motor("shooter-wheel")
                    .voltage(Volts.of(rollers.getVoltage()))
                    .angularPosition(Rotations.of(rollers.getPosition()))
                    .angularVelocity(rollers.getVelocity());
              },
              rollers)
        );

        this.rollers = rollers;
    }

    public Command SysIDQuasistatic(Direction direction){
        return routine.quasistatic(direction);
    }

    public Command SysIDDynamic(Direction direction){
        return routine.dynamic(direction);
    }

    public SequentialCommandGroup doAllSysID(){
        SequentialCommandGroup sysId = new SequentialCommandGroup(
            this.SysIDQuasistatic(Direction.kForward),
            new RunCommand(()->rollers.set(0), rollers).until(()->Math.abs(rollers.getVelocity().in(RPM)) < 50),
            this.SysIDQuasistatic(Direction.kReverse),
            new RunCommand(()->rollers.set(0), rollers).until(()->Math.abs(rollers.getVelocity().in(RPM)) < 50),
            this.SysIDDynamic(Direction.kForward),
            new RunCommand(()->rollers.set(0), rollers).until(()->Math.abs(rollers.getVelocity().in(RPM)) < 50),
            this.SysIDDynamic(Direction.kReverse),
            new RunCommand(()->rollers.set(0), rollers).until(()->Math.abs(rollers.getVelocity().in(RPM)) < 50)
        );
        return sysId;
    }
}
