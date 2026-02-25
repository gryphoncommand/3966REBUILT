package frc.robot.commands.Indexing;

import frc.robot.subsystems.Indexer.Kicker;
import frc.robot.subsystems.Indexer.PreIndexer;
import frc.robot.subsystems.Indexer.Spindexer;

public class FeedShooterFactory {
    private Kicker kicker;
    private PreIndexer preIndexer;
    private Spindexer spindexer;
    private boolean running = false;

    public FeedShooterFactory(Kicker kicker, PreIndexer preIndexer, Spindexer spindexer){
        this.kicker = kicker;
        this.preIndexer = preIndexer;
        this.spindexer = spindexer;
    }
    
    public void start(boolean spindexerDirection){
        running = true;
        kicker.setVelocity(100);
        preIndexer.setVelocity(200);
        double spindexerSpeed = spindexerDirection ? 300 : -300;
        spindexer.setVelocity(spindexerSpeed);
    }

    public void stop(){
        kicker.setVelocity(0);
        preIndexer.setVelocity(0);
        spindexer.setVelocity(0);
        running = false;
    }
    
    public void periodic(){
        if (running){
            if (preIndexer.getTargetRPM() != 200){
                preIndexer.setVelocity(200);
            }
        }
    }
}
