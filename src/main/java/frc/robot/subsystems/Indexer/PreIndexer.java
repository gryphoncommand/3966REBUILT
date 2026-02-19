package frc.robot.subsystems.Indexer;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

public class PreIndexer extends SubsystemBase {
    // Should run whenever spindexer or intake rollers are running
    private TalonFX preIndexerMotor = new TalonFX(IndexerConstants.kPreIndexerCanID);
}
