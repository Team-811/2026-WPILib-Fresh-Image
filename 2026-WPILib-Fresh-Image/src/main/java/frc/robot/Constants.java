package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * Centralized constants and enums. Organize bus names, CAN IDs, and kinematic dimensions here.
 * Many values are placeholders—adjust them to your robot.
 *
 * Notes & reasoning (read this once, it’ll save you time):
 *  - The Kraken X60 integrates a TalonFX motor/controller. We use Phoenix 6 closed-loop with
 *    voltage requests for both drive (velocity) and steer (position), so the Rio only sends
 *    what to do, not how hard to push in amps. Phoenix handles the motor control loop.
 *  - Swerve geometry values (track width & wheelbase) must match your real chassis; they set
 *    the kinematics used to translate chassis speeds into per-module wheel speeds/angles.
 *  - Two modes: COMPETITION_MODE uses full speed, DEMO_MODE scales translational & angular
 *    speeds to 50%. We expose a SendableChooser and a controller button to switch on the fly.
 *  - CAN bus: if you use a CANivore, give it a name (common: "CANivore"). We pass that name
 *    when constructing CTRE devices so they’re placed on that bus instead of the RoboRIO bus.
 */
public final class Constants {

  // ------------------ Robot Modes ------------------
  public enum RobotMode {
    COMPETITION_MODE,
    DEMO_MODE
  }

  // Default at boot; you can change via dashboard or a button
  public static final RobotMode DEFAULT_MODE = RobotMode.COMPETITION_MODE;

  // Speed scaling for demo mode (50%)
  public static final double DEMO_TRANSLATION_SCALE = 0.50;
  public static final double DEMO_ANGULAR_SCALE     = 0.50;

  // ------------------ CAN / Bus Names ------------------
  // If your CANivore is named something else, update this.
  public static final String CANIVORE_NAME = "CANivore";

  // ------------------ Drivetrain Geometry ------------------
  // Measure your robot! These are examples.
  public static final double TRACK_WIDTH_METERS = Units.inchesToMeters(22.0);  // left-to-right
  public static final double WHEELBASE_METERS   = Units.inchesToMeters(22.0);  // front-to-back

  // Wheel info (adjust to your swerve module + wheel)
  public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4.0);
  public static final double WHEEL_CIRCUMFERENCE   = Math.PI * WHEEL_DIAMETER_METERS;

  // Drive gearing (motor rotations per wheel rotation). Placeholder—adjust to your module.
  public static final double DRIVE_GEAR_RATIO = 6.75;

  // Steer gearing (motor rotations per azimuth rotation). Placeholder—adjust to your module.
  public static final double STEER_GEAR_RATIO = 12.8;

  // Max robot speeds (tune on a practice field)
  public static final double MAX_TRANSLATION_MPS   = 4.5; // meters per second
  public static final double MAX_ANGULAR_RPS       = Math.PI; // rad/sec (~180 deg/s)

  // Deadbands for joystick input
  public static final double DEADBAND = 0.08;

  // Slew rate limits to smooth driver inputs
  public static final double XY_SLEW_RATE = 3.0;     // m/s per second
  public static final double ROT_SLEW_RATE = 6.0;    // rad/s per second

  // ------------------ Module Positions (relative to robot center) ------------------
  // +X: forward (toward front bumper), +Y: left
  public static final Translation2d FRONT_LEFT_LOCATION  =
      new Translation2d(+WHEELBASE_METERS / 2.0, +TRACK_WIDTH_METERS / 2.0);
  public static final Translation2d FRONT_RIGHT_LOCATION =
      new Translation2d(+WHEELBASE_METERS / 2.0, -TRACK_WIDTH_METERS / 2.0);
  public static final Translation2d BACK_LEFT_LOCATION   =
      new Translation2d(-WHEELBASE_METERS / 2.0, +TRACK_WIDTH_METERS / 2.0);
  public static final Translation2d BACK_RIGHT_LOCATION  =
      new Translation2d(-WHEELBASE_METERS / 2.0, -TRACK_WIDTH_METERS / 2.0);

  // ------------------ CAN IDs (example placeholders) ------------------
  // TalonFX per module: drive + steer. If you have CANcoders, add those IDs too.
  public static final class CanIds {
    // Front Left
    public static final int FL_DRIVE = 1;
    public static final int FL_STEER = 2;
    public static final int FL_CANCODER = 11;  // optional; set to -1 if not used

    // Front Right
    public static final int FR_DRIVE = 3;
    public static final int FR_STEER = 4;
    public static final int FR_CANCODER = 12;

    // Back Left
    public static final int BL_DRIVE = 5;
    public static final int BL_STEER = 6;
    public static final int BL_CANCODER = 13;

    // Back Right
    public static final int BR_DRIVE = 7;
    public static final int BR_STEER = 8;
    public static final int BR_CANCODER = 14;

    // Pigeon2
    public static final int PIGEON2 = 9;
  }

  private Constants() {}
}
