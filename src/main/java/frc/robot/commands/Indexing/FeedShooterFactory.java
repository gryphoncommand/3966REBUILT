package frc.robot.commands.Indexing;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.IndexerConstants;
import frc.robot.subsystems.Indexer.Kicker;
import frc.robot.subsystems.Indexer.PreIndexer;
import frc.robot.subsystems.Indexer.Spindexer;

public class FeedShooterFactory {
    private Kicker kicker;
    private PreIndexer preIndexer;
    private Spindexer spindexer;
    private boolean running = false;
    private boolean spindexerDirection;
    private final Timer timer = new Timer();

    public FeedShooterFactory(Kicker kicker, PreIndexer preIndexer, Spindexer spindexer){
        this.kicker = kicker;
        this.preIndexer = preIndexer;
        this.spindexer = spindexer;
    }
    
    public void start(boolean spindexerDirection){
        this.spindexerDirection = spindexerDirection;
        running = true;
        kicker.setVelocity(IndexerConstants.kKickerSpeed);
        kicker.set(1);
        preIndexer.setVelocity(IndexerConstants.kPreIndexerSpeed);
        double spindexerSpeed = spindexerDirection ? IndexerConstants.kSpindexerSpeed : -IndexerConstants.kSpindexerSpeed;
        spindexer.setVelocity(spindexerSpeed);
        timer.restart();
    }

    public void stop(){
        kicker.set(0);
        preIndexer.set(0);
        spindexer.set(0);
        running = false;
    }
    
    public void periodic(){
        if (running){
            if (preIndexer.getTargetRPM() != IndexerConstants.kPreIndexerSpeed){
                preIndexer.setVelocity(IndexerConstants.kPreIndexerSpeed);
            }
            if (timer.get() > 0.5 && spindexer.getStatorCurrent() > 50){
                spindexerDirection = !spindexerDirection;
                start(spindexerDirection);
            }
            SmartDashboard.putNumber("Spindexer Stator Current", spindexer.getStatorCurrent());
        }
    }
}
