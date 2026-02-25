package frc.robot.subsystems.Indexer;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.PreIndexerConfig;
import frc.robot.Constants.IndexerConstants;

public class PreIndexer extends SubsystemBase {
    // Should run whenever spindexer or intake rollers are running
    private TalonFX preIndexerMotor = new TalonFX(IndexerConstants.kPreIndexerCanID);

    private double targetRPM = 0.0;
    private VelocityVoltage m_preIndexerRequest = new VelocityVoltage(300);

    public PreIndexer(){
        preIndexerMotor.getConfigurator().apply(PreIndexerConfig.PreIndexerConfig);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("PreIndexer RPM", preIndexerMotor.getVelocity().getValue().in(RPM));
        SmartDashboard.putNumber("Desired PreIndexer RPM", targetRPM);
    }

    public void set(double speed) {
        preIndexerMotor.set(speed);
    }

    public void setVelocity(double rpm){
        m_preIndexerRequest.withVelocity(rpm/60);
        targetRPM = rpm;
        preIndexerMotor.setControl(m_preIndexerRequest);
    }

    public AngularVelocity getVelocity(){
        return preIndexerMotor.getVelocity().getValue();
    }

    public double getTargetRPM(){
        return targetRPM;
    }

    public boolean atTarget(double threshold){
        return Math.abs(getVelocity().in(RPM) - targetRPM) < threshold;
    }
}
