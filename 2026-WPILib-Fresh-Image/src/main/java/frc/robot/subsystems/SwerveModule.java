package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import frc.robot.Constants;

/**
 * A single swerve module (drive + steer + optional CANcoder).
 *
 * Implementation choices explained:
 *  - We use Phoenix 6 voltageclosed-loop requests (VelocityVoltage for drive and
 *    PositionVoltage for steer). These are robust and easy to reason about.
 *  - We support an absolute CANcoder to establish the steer angle at boot; if you rely on
 *    integrated TalonFX sensor only, you’ll need to mechanically align or use Phoenix 6’s
 *    absolute initialization features. Keeping CANcoder here is the least-surprise approach.
 *  - We optimize the desired state to minimize wheel rotation (WPILib helper).
 *  - In sim, we update the CTRE SimState objects so Phoenix client code sees realistic values.
 */
public class SwerveModule {

  // Hardware
  private final TalonFX m_drive;
  private final TalonFX m_steer;
  private final CANcoder m_cancoder;   // may be null if you don’t have one

  // Control requests (reused to reduce allocs)
  private final VelocityVoltage m_driveRequest = new VelocityVoltage(0).withEnableFOC(true);
  private final PositionVoltage m_steerRequest = new PositionVoltage(0).withEnableFOC(true);

  // Sim states
  private final TalonFXSimState m_driveSim;
  private final TalonFXSimState m_steerSim;
  private final CANcoderSimState m_cancoderSim;

  // Feedforward for drive (tune ks, kv, ka)
  private final SimpleMotorFeedforward m_ff = new SimpleMotorFeedforward(0.1, 2.2, 0.2);

  // Conversions
  private final double m_driveRotPerMeter; // motor rotations per meter of wheel travel
  private final double m_steerRotPerRad;   // motor rotations per radian of azimuth rotation

  // Optional steer absolute offset (radians): measure & set per module!
  private final double m_steerOffsetRad;
  
  // --- SIM accumulators ---
  private double m_simDriveRotorPos = 0.0;   // motor rotations

  /**
   * @param driveId       TalonFX CAN ID (drive)
   * @param steerId       TalonFX CAN ID (steer)
   * @param cancoderId    CANcoder ID, or -1 if not used
   * @param steerOffsetRad Absolute angle offset of the module’s zero, in radians
   * @param busName       CAN bus name (e.g. CANivore), or null for default
   */
  public SwerveModule(int driveId, int steerId, int cancoderId, double steerOffsetRad, String busName) {
    m_steerOffsetRad = steerOffsetRad;

    m_drive = (busName == null) ? new TalonFX(driveId) : new TalonFX(driveId, busName);
    m_steer = (busName == null) ? new TalonFX(steerId) : new TalonFX(steerId, busName);
    m_cancoder = (cancoderId >= 0) ? ((busName == null) ? new CANcoder(cancoderId)
                                                        : new CANcoder(cancoderId, busName))
                                   : null;

    // Configure motors
    TalonFXConfiguration driveCfg = new TalonFXConfiguration();
    driveCfg.Slot0 = new Slot0Configs()
        .withKP(0.2).withKI(0.0).withKD(0.0)   // start conservative; tune later
        .withKV(0.12);                         // Phoenix feedforward slope (approx)
    driveCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    m_drive.getConfigurator().apply(driveCfg);

    TalonFXConfiguration steerCfg = new TalonFXConfiguration();
    steerCfg.Slot0 = new Slot0Configs()
        .withKP(12.0).withKI(0.0).withKD(0.1); // steer needs snappy response
    steerCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    m_steer.getConfigurator().apply(steerCfg);

    if (m_cancoder != null) {
      CANcoderConfiguration cc = new CANcoderConfiguration();
      cc.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
      m_cancoder.getConfigurator().apply(cc);
    }

    // Precompute conversion factors
    // Motor rotations per meter = (gear ratio) / (wheel circumference in meters)
    m_driveRotPerMeter = Constants.DRIVE_GEAR_RATIO / Constants.WHEEL_CIRCUMFERENCE;
    // Motor rotations per radian of steer = (gear ratio) / (2*pi radians per revolution)
    m_steerRotPerRad = Constants.STEER_GEAR_RATIO / (2.0 * Math.PI);

    // SIM states
    m_driveSim = m_drive.getSimState();
    m_steerSim = m_steer.getSimState();
    m_cancoderSim = (m_cancoder != null) ? m_cancoder.getSimState() : null;

    // Speed up status frames just enough for smooth odometry without spamming CAN
    BaseStatusSignal.setUpdateFrequencyForAll(100,  // Hz
        m_drive.getVelocity(), m_drive.getPosition(),
        m_steer.getPosition());
  }

  /** Read the current module angle (Rotation2d), using steer motor integrated sensor. */
  public Rotation2d getAngle() {
    // Convert steer motor rotations into radians, subtract the known mechanical offset.
    double steerMotorRot = m_steer.getPosition().getValueAsDouble();
    double angleRad = (steerMotorRot / m_steerRotPerRad) - m_steerOffsetRad;
    return new Rotation2d(angleRad);
  }

  /** Read the current wheel speed in meters per second from drive motor velocity. */
  public double getWheelSpeedMetersPerSec() {
    // TalonFX velocity default is rotations per second of the motor (Phoenix 6)
    double motorRps = m_drive.getVelocity().getValueAsDouble();
    double wheelMps = (motorRps / m_driveRotPerMeter);
    return wheelMps;
  }

  /**
   * Set desired module state with optimization to minimize rotation.
   * We command:
   *  - steer: absolute azimuth angle (PositionVoltage with FOC)
   *  - drive: wheel speed closed-loop (VelocityVoltage + arbitrary feedforward)
   */
  public void setDesiredState(SwerveModuleState desired) {
    // Optimize state relative to current module angle
    SwerveModuleState optimized =
        SwerveModuleState.optimize(desired, getAngle());

    // Steer: convert target angle (rad) into motor rotations
    double targetAngleRad = optimized.angle.getRadians() + m_steerOffsetRad;
    double steerMotorRot = targetAngleRad * m_steerRotPerRad;

    // Drive: convert wheel m/s to motor rps
    double wheelMps = optimized.speedMetersPerSecond;
    double motorRps = wheelMps * m_driveRotPerMeter;

    // Feedforward for drive (volts)
    double ffVolts = m_ff.calculate(wheelMps);

    // Send to motors
    m_steer.setControl(m_steerRequest.withPosition(steerMotorRot));
    m_drive.setControl(m_driveRequest.withVelocity(motorRps).withFeedForward(ffVolts));
  }

  // ---------------- Simulation helpers ----------------

  /** Called from the subsystem’s simulationPeriodic with dt seconds. */
  public void updateSim(double dtSeconds) {
    // Steer: move toward demanded position instantly (simple but workable)
    double targetSteerRot = m_steerRequest.Position; // what we last commanded
    m_steerSim.setRawRotorPosition(targetSteerRot);
    // crude velocity consistent with change
    m_steerSim.setRotorVelocity(0);

    // Drive: set velocity to requested closed-loop target (idealized)
    double targetMotorRps = m_driveRequest.Velocity;
    m_driveSim.setRotorVelocity(targetMotorRps);

    // Integrate rotor position so odometry can read distance in sim
    m_simDriveRotorPos += targetMotorRps * dtSeconds;  // rotations += rps * s
    m_driveSim.setRawRotorPosition(m_simDriveRotorPos);


    // If you have a CANcoder, mirror the absolute angle (with offset)
    if (m_cancoderSim != null) {
      double absAngleRad = (targetSteerRot / m_steerRotPerRad);
      m_cancoderSim.setRawPosition(absAngleRad / (2.0 * Math.PI)); // turns
      m_cancoderSim.setVelocity(0);
    }
  }

  /** Return the current module state (speed + angle) for odometry/pose estimation. */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getWheelSpeedMetersPerSec(), getAngle());
  }
  /** Total wheel distance traveled in meters (from drive motor rotations). */
  public double getWheelDistanceMeters() {
    double motorRot = m_drive.getPosition().getValueAsDouble();   // TalonFX integrated position (rotations)
    return motorRot / m_driveRotPerMeter;                          // meters = motorRot / (rot/meter)
  }
  
  /** Module position = (distance meters, current azimuth). Used by odometry. */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getWheelDistanceMeters(), getAngle());
  }
  
}
