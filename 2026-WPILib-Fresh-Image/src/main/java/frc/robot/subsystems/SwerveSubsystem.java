package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
//import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import frc.robot.Constants;
//import frc.robot.Constants.CanIds;
//import frc.robot.Constants.RobotMode;

public class SwerveSubsystem extends SubsystemBase {

  private final SwerveModule m_frontLeft =
      new SwerveModule(
        Constants.CanIds.FL_DRIVE, Constants.CanIds.FL_STEER, Constants.CanIds.FL_CANCODER,
        0.0 /* steer offset rad: measure per module */, Constants.CANIVORE_NAME);

  private final SwerveModule m_frontRight =
      new SwerveModule(
        Constants.CanIds.FR_DRIVE, Constants.CanIds.FR_STEER, Constants.CanIds.FR_CANCODER,
        0.0, Constants.CANIVORE_NAME);

  private final SwerveModule m_backLeft =
      new SwerveModule(
        Constants.CanIds.BL_DRIVE, Constants.CanIds.BL_STEER, Constants.CanIds.BL_CANCODER,
        0.0, Constants.CANIVORE_NAME);

  private final SwerveModule m_backRight =
      new SwerveModule(
        Constants.CanIds.BR_DRIVE, Constants.CanIds.BR_STEER, Constants.CanIds.BR_CANCODER,
        0.0, Constants.CANIVORE_NAME);

  private final Pigeon2 m_pigeon = new Pigeon2(Constants.CanIds.PIGEON2, Constants.CANIVORE_NAME);
  private final Pigeon2SimState m_pigeonSim = m_pigeon.getSimState();

  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          Constants.FRONT_LEFT_LOCATION,
          Constants.FRONT_RIGHT_LOCATION,
          Constants.BACK_LEFT_LOCATION,
          Constants.BACK_RIGHT_LOCATION);

  //private final SwerveDriveOdometry m_odometry =
  //    new SwerveDriveOdometry(m_kinematics, getYaw(), getModulePositions());
  // replace m_odometry with:
  private final SwerveDrivePoseEstimator m_pose =
      new SwerveDrivePoseEstimator(m_kinematics, getYaw(), getModulePositions(), new Pose2d());
        
  // Add Field2d so you can see the robot in sim/real
  private final Field2d m_field = new Field2d();

  // Driver input conditioning
  private final SlewRateLimiter m_xLimiter = new SlewRateLimiter(Constants.XY_SLEW_RATE);
  private final SlewRateLimiter m_yLimiter = new SlewRateLimiter(Constants.XY_SLEW_RATE);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(Constants.ROT_SLEW_RATE);
  
  private Constants.RobotMode m_mode = Constants.DEFAULT_MODE;

  public SwerveSubsystem() {    
    zeroHeading(); // zero the gyro at startup for field-relative driving
    SmartDashboard.putData("Field", m_field); //construct Field
  }

  private SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      m_frontLeft.getPosition(),
      m_frontRight.getPosition(),
      m_backLeft.getPosition(),
      m_backRight.getPosition()
    };
  }

  // ---------------- Driver control ----------------

  /**
   * Drive the robot with requested chassis speeds (field-relative or robot-relative).
   * -  xSpeed  forward m/s (+X)
   * -  ySpeed  left m/s (+Y)
   * -  omega   CCW rad/s
   * -  fieldRelative true for field-relative using Pigeon2 yaw
   */
  public void drive(double xSpeed, double ySpeed, double omega, boolean fieldRelative) {
    // Mode scaling (DEMO_MODE halves max outputs)
    double transScale = (m_mode == Constants.RobotMode.DEMO_MODE)
        ? Constants.DEMO_TRANSLATION_SCALE : 1.0;
    double rotScale = (m_mode == Constants.RobotMode.DEMO_MODE)
        ? Constants.DEMO_ANGULAR_SCALE : 1.0;

    // Clamp to max speed then apply slew rate for smoother response
    double xCmd = m_xLimiter.calculate(
        clamp(xSpeed, -Constants.MAX_TRANSLATION_MPS, Constants.MAX_TRANSLATION_MPS) * transScale);
    double yCmd = m_yLimiter.calculate(
        clamp(ySpeed, -Constants.MAX_TRANSLATION_MPS, Constants.MAX_TRANSLATION_MPS) * transScale);
    double rCmd = m_rotLimiter.calculate(
        clamp(omega, -Constants.MAX_ANGULAR_RPS, Constants.MAX_ANGULAR_RPS) * rotScale);

    SwerveModuleState[] states =
        (fieldRelative)
            ? m_kinematics.toSwerveModuleStates(
                  edu.wpi.first.math.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(
                      xCmd, yCmd, rCmd, getYaw()))
            : m_kinematics.toSwerveModuleStates(
                  new edu.wpi.first.math.kinematics.ChassisSpeeds(xCmd, yCmd, rCmd));

    // Desaturate to respect max wheel speed
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.MAX_TRANSLATION_MPS);

    // Send to modules
    m_frontLeft.setDesiredState(states[0]);
    m_frontRight.setDesiredState(states[1]);
    m_backLeft.setDesiredState(states[2]);
    m_backRight.setDesiredState(states[3]);
  }

  /** Quick helper for joystick deadband. */
  public static double applyDeadband(double val, double deadband) {
    return (Math.abs(val) > deadband) ? val : 0.0;
  }

  /** Clamp a value between min and max. */
  private static double clamp(double v, double min, double max) {
    return Math.max(min, Math.min(max, v));
  }

  // ---------------- Odometry / Pose ----------------

  public Rotation2d getYaw() {
    // Pigeon2 yaw positive-CCW; WPILib uses CCW-positive as well.
    return Rotation2d.fromDegrees(m_pigeon.getYaw().getValueAsDouble());
  }

  public void zeroHeading() {
    m_pigeon.reset();
  }

  
  // getters/updates for pose estimator
  public Pose2d getPose() { return m_pose.getEstimatedPosition(); }
  
  public void resetPose(Pose2d pose) {
    m_pose.resetPosition(getYaw(), getModulePositions(), pose);
  }

  @Override
  public void periodic() {
    // Update pose estimator.
    m_pose.update(getYaw(), getModulePositions());
    
    // Update field visualization
    m_field.setRobotPose(getPose());    
  }

  // ---------------- Simulation ----------------
  // We integrate commanded wheel states into a simple chassis model,
  // update the pose, and reflect yaw into the Pigeon2 sim.

  private Pose2d m_simPose = new Pose2d();

  @Override
  public void simulationPeriodic() {
    final double dt = 0.02; // 20ms typical

    // Let each module update its TalonFX/CANcoder sim signals
    m_frontLeft.updateSim(dt);
    m_frontRight.updateSim(dt);
    m_backLeft.updateSim(dt);
    m_backRight.updateSim(dt);

    // Approximate chassis motion from current module states
    SwerveModuleState[] states = new SwerveModuleState[] {
      m_frontLeft.getState(), m_frontRight.getState(),
      m_backLeft.getState(),  m_backRight.getState()
    };

    var chassisSpeeds = m_kinematics.toChassisSpeeds(states);

    // Integrate pose (very lightweight model; good enough for testing code paths)
    double dx = chassisSpeeds.vxMetersPerSecond * dt;
    double dy = chassisSpeeds.vyMetersPerSecond * dt;
    double dtheta = chassisSpeeds.omegaRadiansPerSecond * dt;

    m_simPose = new Pose2d(
        m_simPose.getX() + (dx * Math.cos(m_simPose.getRotation().getRadians())
                          - dy * Math.sin(m_simPose.getRotation().getRadians())),
        m_simPose.getY() + (dx * Math.sin(m_simPose.getRotation().getRadians())
                          + dy * Math.cos(m_simPose.getRotation().getRadians())),
        m_simPose.getRotation().plus(new Rotation2d(dtheta)));

    // Push sim yaw to Pigeon
    m_pigeonSim.setRawYaw(m_simPose.getRotation().getDegrees());
    m_pigeonSim.setAngularVelocityZ(chassisSpeeds.omegaRadiansPerSecond * (180.0/Math.PI));
  }

  // ---------------- Mode control ----------------

  public void setMode(Constants.RobotMode mode) {
    m_mode = mode;
  }

  public Constants.RobotMode getMode() {
    return m_mode;
  }
}
