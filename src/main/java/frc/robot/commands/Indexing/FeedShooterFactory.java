package frc.robot.commands.Indexing;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.IndexerConstants;
import frc.robot.subsystems.Indexer.Kicker;
import frc.robot.subsystems.Indexer.PreIndexer;

public class FeedShooterFactory {
    private Kicker kicker;
    private PreIndexer preIndexer;
    private boolean running = false;
    private final Timer timer = new Timer();

    public FeedShooterFactory(Kicker kicker, PreIndexer preIndexer){
        this.kicker = kicker;
        this.preIndexer = preIndexer;
    }
    
    public void start(){
        running = true;
        kicker.setVelocity(IndexerConstants.kKickerSpeed);
        kicker.set(-0.95);
        preIndexer.setVelocity(IndexerConstants.kPreIndexerSpeed);
        timer.restart();
    }

    public void stop(){
        kicker.setVelocity(0);
        kicker.set(0);
        preIndexer.setVelocity(0);
        preIndexer.set(0);
        running = false;
    }
    
    public void periodic(){
        if (running){
            if (preIndexer.getTargetRPM() != IndexerConstants.kPreIndexerSpeed){
                preIndexer.setVelocity(IndexerConstants.kPreIndexerSpeed);
            }
        }
    }
}
