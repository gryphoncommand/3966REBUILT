package frc.GryphonLib;

public class ShooterState {
  public final double distanceMeters;
  public final double hoodAngleDeg;
  public final double flywheelRPM;

  public ShooterState(double distanceMeters,
                      double hoodAngleDeg,
                      double flywheelRPM) {
    this.distanceMeters = distanceMeters;
    this.hoodAngleDeg = hoodAngleDeg;
    this.flywheelRPM = flywheelRPM;
  }
}
