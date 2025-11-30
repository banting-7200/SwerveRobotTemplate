package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Utilites.Constants;
import frc.robot.Utilites.LEDRequest;
import frc.robot.Utilites.LEDRequest.LEDState;

public class Robot extends TimedRobot {

  private static Robot instance;
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private Timer disabledTimer;

  public Robot() {
    instance = this;
  }

  public static Robot getInstance() {
    return instance;
  }

  @Override
  public void robotInit() {

    m_robotContainer = RobotContainer.getInstance();

    // Create a timer to disable motor brake a few seconds after disable. This will
    // let the robot stop
    // immediately when disabled, but then also let it be pushed more
    disabledTimer = new Timer();
  }

  
  @Override
  public void robotPeriodic() {
    m_robotContainer.robotPerodic();
    m_robotContainer.updateTelemetry();
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
   // m_robotContainer.setMotorBrake(true);
    disabledTimer.reset();
    disabledTimer.start();
  }

  @Override
  public void disabledPeriodic() {
    // if (disabledTimer.hasElapsed(Constants.DrivebaseConstants.WHEEL_LOCK_TIME)) {
    //   m_robotContainer.setMotorBrake(false);
    //   disabledTimer.stop();
    //   disabledTimer.reset();
    // }
    m_robotContainer.lights.requestLEDState(new LEDRequest(LEDState.RAINBOW));;
  }


  @Override
  public void autonomousInit() {
    m_robotContainer.setDashboardTab("Autonomous");
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }


  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    m_robotContainer.setDashboardTab("Teleoperated");
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    } else {
      CommandScheduler.getInstance().cancelAll();
    }
    m_robotContainer.lights.requestLEDState(new LEDRequest(LEDState.SOLID).withColour(Color.kGreen));
  }

  @Override
  public void teleopPeriodic() {
    m_robotContainer.enabledPerodic();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}