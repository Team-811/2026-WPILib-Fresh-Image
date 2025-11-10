package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.subsystems.SwerveSubsystem;

/**
 * Wires up subsystems, default commands, and driver controls.
 *
 * Control scheme:
 *   - Left stick: translation X/Y (field-relative)
 *   - Right stick X: rotation (CCW positive)
 *   - A button: zero heading (re-zero Pigeon)
 *   - Y button: toggle COMP/DEMO mode on the fly
 *   - Dashboard chooser also lets you pick mode at boot
 */
public class RobotContainer {

  private final SwerveSubsystem m_swerve = new SwerveSubsystem();
  private final XboxController m_driver = new XboxController(0);

  private final SendableChooser<Constants.RobotMode> m_modeChooser = new SendableChooser<>();

  public RobotContainer() {
    configureModeChooser();
    configureDefaultDrive();
    configureBindings();
  }

  private void configureModeChooser() {
    m_modeChooser.setDefaultOption("Competition (Full)", Constants.RobotMode.COMPETITION_MODE);
    m_modeChooser.addOption("Demo (50%)", Constants.RobotMode.DEMO_MODE);
    SmartDashboard.putData("Robot Mode", m_modeChooser);
  }

  private void configureDefaultDrive() {
    m_swerve.setDefaultCommand(new RunCommand(() -> {
      // Read sticks and apply deadband
      double lx = -m_driver.getLeftY(); // invert so up is +X forward
      double ly = -m_driver.getLeftX(); // invert so right stick left is +Y left
      double rx = -m_driver.getRightX(); // CCW positive

      lx = SwerveSubsystem.applyDeadband(lx, Constants.DEADBAND);
      ly = SwerveSubsystem.applyDeadband(ly, Constants.DEADBAND);
      rx = SwerveSubsystem.applyDeadband(rx, Constants.DEADBAND);

      // Map to max speed
      double xSpeed  = lx * Constants.MAX_TRANSLATION_MPS;
      double ySpeed  = ly * Constants.MAX_TRANSLATION_MPS;
      double omega   = rx * Constants.MAX_ANGULAR_RPS;

      m_swerve.drive(xSpeed, ySpeed, omega, /*fieldRelative*/ true);
    }, m_swerve));
  }


/*
| Control              | Function                                          | Notes / Behavior                                                                             |
| ---------------------| ------------------------------------------------- | ---------------------------------------------------------------------------------------------|
| Left Stick (Y-axis)  | Forward / Backward drive                          | Pushing up drives forward (+X field axis).                                                   |
| Left Stick (X-axis)  | Strafe left / right                               | Pushing right strafes robot right (+Y field axis).                                           |
| Right Stick (X-axis) | Rotate robot (turn)                               | Right = clockwise, Left = counterclockwise.                                                  |
| A Button             | Zero heading (reset gyro)                         | Calls m_swerve.zeroHeading(), sets Pigeon2 yaw to 0. Useful if field-relative control drifts.|
| Y Button             | Toggle mode between COMPETITION_MODE and DEMO_MOD | Switches between full-speed and 50%-speed operation instantly. Works live during teleop.     |
| (Other buttons)      | —                                                 | Currently unused — available for future features (e.g. auto align, intake, shoot).           |
*/
  private void configureBindings() {
    // A → zero heading
    new edu.wpi.first.wpilibj2.command.button.JoystickButton(m_driver, XboxController.Button.kA.value)
        .onTrue(new InstantCommand(() -> {
        m_swerve.zeroHeading();
        m_swerve.resetPose(new Pose2d());
    }, m_swerve));

    // Y → toggle mode on the fly
    new edu.wpi.first.wpilibj2.command.button.JoystickButton(m_driver, XboxController.Button.kY.value)
        .onTrue(new edu.wpi.first.wpilibj2.command.InstantCommand(() -> {
          var current = m_swerve.getMode();
          m_swerve.setMode(
              (current == Constants.RobotMode.COMPETITION_MODE)
                ? Constants.RobotMode.DEMO_MODE
                : Constants.RobotMode.COMPETITION_MODE);
        }, m_swerve));
  }

  /** Called by Robot.teleopInit to apply chooser selection at enable. */
  public void applyChosenMode() {
    m_swerve.setMode(m_modeChooser.getSelected());
  }

  public SwerveSubsystem getSwerve() {
    return m_swerve;
  }
}
