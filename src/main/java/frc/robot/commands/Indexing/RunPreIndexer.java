package frc.robot.commands.Indexing;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Indexer.PreIndexer;

public class RunPreIndexer extends Command {
    PreIndexer preIndexer;

    public RunPreIndexer(PreIndexer preIndexer){
        this.preIndexer = preIndexer;

        addRequirements(preIndexer);
    }

    @Override
    public void initialize() {
        setName("Pre-Indexer Running");
        preIndexer.setVelocity(200);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        preIndexer.setVelocity(0);
    }
}
