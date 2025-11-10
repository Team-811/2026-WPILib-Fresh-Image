package frc.robot.subsystems;

public final class SwerveUtils {
  private SwerveUtils() {}

  /** Apply a deadband to a joystick input and preserve sign. */
  public static double deadband(double x, double deadband) {
    if (Math.abs(x) < deadband) return 0.0;
    return (x - Math.copySign(deadband, x)) / (1.0 - deadband);
  }

  /** Square the input (for finer control near center) while preserving sign. */
  public static double squarePreserveSign(double x) {
    return Math.copySign(x * x, x);
  }
}
