package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * Thin Robot class that delegates to command framework.
 * We use simulationPeriodic to drive the lightweight swerve physics.
 */
public class Robot extends TimedRobot {

  private RobotContainer m_container;

  @Override
  public void robotInit() {
    m_container = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    // You can schedule autonomous commands here later
  }

  @Override
  public void teleopInit() {
    // Apply the dashboard mode selection when teleop starts
    m_container.applyChosenMode();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void simulationPeriodic() {
    // Nothing needed here; subsystem’s simulationPeriodic handles physics
  }
}
