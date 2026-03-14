package frc.robot.commands;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.Flywheel.FlywheelIO;

public class FlywheelSysID {    
    SysIdRoutine routine;
    FlywheelIO flywheel;

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

        this.flywheel = flywheel;
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
            new RunCommand(()->flywheel.set(0), flywheel).until(()->Math.abs(flywheel.getVelocity()) < 50),
            this.SysIDQuasistatic(Direction.kReverse),
            new RunCommand(()->flywheel.set(0), flywheel).until(()->Math.abs(flywheel.getVelocity()) < 50),
            this.SysIDDynamic(Direction.kForward),
            new RunCommand(()->flywheel.set(0), flywheel).until(()->Math.abs(flywheel.getVelocity()) < 50),
            this.SysIDDynamic(Direction.kReverse),
            new RunCommand(()->flywheel.set(0), flywheel).until(()->Math.abs(flywheel.getVelocity()) < 50)
        );
        return sysId;
    }
}
