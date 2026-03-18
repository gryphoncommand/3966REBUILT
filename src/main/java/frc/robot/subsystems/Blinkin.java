package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.GryphonLib.HubTracker;
import frc.robot.Constants;
import frc.robot.Constants.BlinkinConstants;

public class Blinkin extends SubsystemBase{
    
    public Spark m_blinkin = new Spark(0);

    public double getColor(){
        return m_blinkin.get();
    }

    public void set(double value){
        m_blinkin.set(value);
    }

    public void periodic() {
        
        if (HubTracker.isAllianceAboutToSwitch()) {
            if (HubTracker.getActiveAlliance().equals(Alliance.Red)) {
                m_blinkin.set(BlinkinConstants.strobeBlue);
            } else {
                m_blinkin.set(BlinkinConstants.strobeRed);
            }
        } else if (HubTracker.areBothAlliancesActive()) {
            m_blinkin.set(BlinkinConstants.white);
        } else {
            if (HubTracker.getActiveAlliance().equals(Alliance.Red)) {
                m_blinkin.set(BlinkinConstants.pink);
            } else {
                m_blinkin.set(BlinkinConstants.blue);
            }
        }

    }

}
