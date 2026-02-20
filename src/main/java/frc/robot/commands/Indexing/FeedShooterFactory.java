package frc.robot.commands.Indexing;

import frc.robot.subsystems.Indexer.Kicker;
import frc.robot.subsystems.Indexer.PreIndexer;
import frc.robot.subsystems.Indexer.Spindexer;

public class FeedShooterFactory {
    private Kicker kicker;
    private PreIndexer preIndexer;
    private Spindexer spindexer;

    public FeedShooterFactory(Kicker kicker, PreIndexer preIndexer, Spindexer spindexer){
        this.kicker = kicker;
        this.preIndexer = preIndexer;
        this.spindexer = spindexer;
    }
    
    public void start(boolean spindexerDirection){
        kicker.setVelocity(100);
        preIndexer.setVelocity(100);
        double spindexerSpeed = spindexerDirection ? 100 : -100;
        spindexer.setVelocity(spindexerSpeed);
    }

    public void stop(){
        kicker.set(0);
        preIndexer.set(0);
        spindexer.set(0);
    }
}
