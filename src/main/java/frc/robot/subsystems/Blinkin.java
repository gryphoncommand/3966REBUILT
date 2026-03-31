package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.littletonUtils.HubShiftUtil;

public class Blinkin extends SubsystemBase {
    public Spark m_blinkin = new Spark(0);

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Blinkin Color", m_blinkin.get());
        if (HubShiftUtil.getShiftedShiftInfo().remainingTime() < 3){
              m_blinkin.set(0.93); // White
        }
        else if (HubShiftUtil.getShiftedShiftInfo().active()) {
            m_blinkin.set(0.57); // Hot Pink
        } else {
            m_blinkin.set(0.83); // Sky Blue
        }
    }

    public Blinkin(){
        m_blinkin.set(0.57);
    }
    

    public void set(double value){
        m_blinkin.set(value);
    }
}