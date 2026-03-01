package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class AlignToTrench extends Command {

    private final DriveSubsystem drive;
    private final CommandXboxController controller;

    private final PIDController yPID = new PIDController(2.5, 0, 0.1);
    private final ProfiledPIDController thetaPID =
        new ProfiledPIDController(
                4.0,
                0,
                0,
                new TrapezoidProfile.Constraints(
                        6.0,  // max angular velocity (rad/s)
                        12.0  // max angular acceleration (rad/s^2)
                )
        );

    public AlignToTrench(DriveSubsystem drive, CommandXboxController controller) {
        this.drive = drive;
        this.controller = controller;

        thetaPID.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drive);

        yPID.setTolerance(0.03);
        thetaPID.setTolerance(0.02);
    }

    @Override
    public void execute() {

        Pose2d pose = drive.getCurrentPose();

        double currentY = pose.getY();
        double currentTheta = pose.getRotation().getRadians();

        // --- Choose closest Y target ---
        double targetY = (Math.abs(currentY - 0.6444996) < Math.abs(currentY - 7.4247756))
                ? 0.6444996
                : 7.4247756;

        // --- Choose closest heading target ---
        double distToZero = Math.abs(MathUtil.angleModulus(currentTheta - 0));
        double distToPi = Math.abs(MathUtil.angleModulus(currentTheta - Math.PI));

        double targetTheta = (distToZero < distToPi)
                ? 0.0
                : Math.PI;

        // --- Driver X control only ---
        double driverX = -controller.getLeftY();
        driverX = MathUtil.applyDeadband(driverX, 0.05) * DriveConstants.kMaxSpeedMetersPerSecond;

        // --- PID Corrections ---
        double yCorrection = yPID.calculate(currentY, targetY);
        double thetaCorrection = thetaPID.calculate(currentTheta, targetTheta);

        if (yPID.atSetpoint()) yCorrection = 0;
        if (thetaPID.atSetpoint()) thetaCorrection = 0;

        // Drive: X from driver, Y + Rot from PID
        drive.driveFieldRelativeChassis(new ChassisSpeeds(driverX, yCorrection, thetaCorrection));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}