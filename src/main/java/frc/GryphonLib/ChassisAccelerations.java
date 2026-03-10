package frc.GryphonLib;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class ChassisAccelerations {
    private double x;
    private double y;
    private double theta;

    public ChassisAccelerations(){
        this(0, 0, 0);
    }

    public ChassisAccelerations(double x, double y, double theta){
        this.theta = theta;
        this.x = x;
        this.y = y;
    }

    public ChassisAccelerations(ChassisSpeeds oldSpeeds, ChassisSpeeds newSpeeds, double dt){
        this.x = (newSpeeds.vxMetersPerSecond - oldSpeeds.vxMetersPerSecond)/dt;
        this.y = (newSpeeds.vyMetersPerSecond - oldSpeeds.vyMetersPerSecond)/dt;
        this.theta = (newSpeeds.omegaRadiansPerSecond - oldSpeeds.omegaRadiansPerSecond)/dt;
    }

    public ChassisSpeeds apply(ChassisSpeeds currentSpeeds, double dt){
        ChassisSpeeds newSpeeds = new ChassisSpeeds(currentSpeeds.vxMetersPerSecond + x*dt, currentSpeeds.vyMetersPerSecond + y*dt, currentSpeeds.omegaRadiansPerSecond + theta*dt);
        return newSpeeds;
    }

    public double getX(){
        return x;
    }

    public double getY(){
        return y;
    }

    public double getTheta(){
        return theta;
    }
}
