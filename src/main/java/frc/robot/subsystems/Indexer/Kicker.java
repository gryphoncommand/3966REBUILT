package frc.robot.subsystems.Indexer;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

public class Kicker extends SubsystemBase {
    private SparkFlex kickerMotor = new SparkFlex(IndexerConstants.kKickerCanID, MotorType.kBrushless);
}
