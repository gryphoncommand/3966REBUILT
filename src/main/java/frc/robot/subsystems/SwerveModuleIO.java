package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.ModuleConstants;

public interface SwerveModuleIO {
    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState();

    public SwerveModuleState getDesiredState();

    /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
    public SwerveModulePosition getPosition();

    /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
    public void setDesiredState(SwerveModuleState desiredState);

    static double mpsToRps(double mps) {
        return (mps / (ModuleConstants.kWheelDiameterMeters * Math.PI)) * ModuleConstants.kDrivingMotorReduction;
    }

    static double rpsToMps(double rps) {
        return rps * ((ModuleConstants.kWheelDiameterMeters * Math.PI) / ModuleConstants.kDrivingMotorReduction);
    }

    /** Zeroes all the SwerveModule encoders. */
    public void resetEncoders();
}