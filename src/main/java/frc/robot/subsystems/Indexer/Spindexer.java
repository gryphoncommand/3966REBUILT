package frc.robot.subsystems.Indexer;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

public class Spindexer extends SubsystemBase {
    private SparkFlex spindexerMotor = new SparkFlex(IndexerConstants.kSpindexerCanID, MotorType.kBrushless);
}
