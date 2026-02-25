package frc.robot.commands;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.Flywheel.FlywheelIO;

public class FlywheelSysID {    
    SysIdRoutine routine;

    public FlywheelSysID(FlywheelIO flywheel){
        routine = new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism((volts) -> flywheel.setVoltage(volts.in(Volts)), 
        log -> {
                // Record a frame for the shooter motor.
                log.motor("shooter-wheel")
                    .voltage(Volts.of(flywheel.getVoltage()))
                    .angularPosition(Rotations.of(flywheel.getPosition()))
                    .angularVelocity(RPM.of(flywheel.getVelocity()));
              },
              flywheel)
        );
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
            this.SysIDQuasistatic(Direction.kReverse),
            this.SysIDDynamic(Direction.kForward),
            this.SysIDDynamic(Direction.kReverse)
        );
        return sysId;
    }
}
