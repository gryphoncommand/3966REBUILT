package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drive.DriveIO;

public class AlignToTrench extends Command {

    private final DriveIO drive;
    private final CommandXboxController controller;
    private double targetY;


    private final SlewRateLimiter xLimiter = new SlewRateLimiter(6);
    private final PIDController yPID = new PIDController(2.5, 0, 0.3);
    private final PIDController thetaPID = new PIDController(4.0, 0, 0.4);

    public AlignToTrench(DriveIO drive, CommandXboxController controller) {
        this.drive = drive;
        this.controller = controller;

        thetaPID.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drive);

        thetaPID.setTolerance(Math.PI/32);
        yPID.setTolerance(0.2);
    }

    @Override
    public void initialize() {
        Pose2d pose = drive.getCurrentPose();
        xLimiter.reset(drive.getCurrentSpeedsFieldRelative().vxMetersPerSecond);
        yPID.reset();
        thetaPID.reset();
        Twist2d trenchTrans = drive.getCurrentSpeedsFieldRelative().toTwist2d(0.5);
        double continuedY = pose.exp(trenchTrans).getY();

        // --- Choose closest Y target ---
        targetY = (Math.abs(continuedY - 0.6444996) < Math.abs(continuedY - 7.4247756))
                ? 0.6444996
                : 7.4247756;
    }

    @Override
    public void execute() {

        Pose2d pose = drive.getCurrentPose();

        double currentY = pose.getY();
        double currentTheta = pose.getRotation().getRadians();


        // --- Choose closest heading target ---
        double distToZero = Math.abs(MathUtil.angleModulus(currentTheta - 0));
        double distToPi = Math.abs(MathUtil.angleModulus(currentTheta - Math.PI));

        double targetTheta = (distToZero < distToPi)
                ? 0.0
                : Math.PI;

        // --- Driver X control only ---
        double driverX = -controller.getLeftY();
        driverX = xLimiter.calculate(MathUtil.applyDeadband(driverX, 0.05) * DriveConstants.kMaxSpeedMetersPerSecond);

        // --- PID Corrections ---
        double yCorrection = yPID.calculate(currentY, targetY);
        
        double thetaCorrection = thetaPID.calculate(currentTheta, targetTheta);

        Logger.recordOutput("Align To Trench Distance", currentY - targetY);
        Logger.recordOutput("Align To Trench Y Correction", yCorrection);
        Logger.recordOutput("Align To Trench Theta Correction", thetaCorrection);
        
        // if (yPID.atSetpoint() && Math.abs(yCorrection) > 0.05){
        //     yCorrection = Math.copySign(0.05, yCorrection);
        // }
        if (thetaPID.atSetpoint()){
            thetaCorrection = 0;
        }

        // Drive: X from driver, Y + Rot from PID
        drive.driveFieldRelativeChassis(new ChassisSpeeds(driverX, yCorrection, thetaCorrection));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}