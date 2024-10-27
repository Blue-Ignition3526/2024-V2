// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.urcl.URCL;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.hal.PowerDistributionVersion;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import lib.BlueShift.control.Buzzer;

public class Robot extends TimedRobot {
  // * Robot Container
  private RobotContainer m_robotContainer;
  
  // * Commands
  private Command m_autonomousCommand;

  // * PDP
  private static final PowerDistribution pdp = new PowerDistribution(1, ModuleType.kRev);

  // * Buzzer
  private static final Buzzer buzzer = new Buzzer(0);

  @Override
  public void robotInit() {
    // * Create robot controller
    m_robotContainer = new RobotContainer();

    // * DISABLE LIVE WINDOW
    LiveWindow.disableAllTelemetry();

    // * DISABLE PHOENIX LOGGING
    SignalLogger.enableAutoLogging(false);
    SignalLogger.stop();

    // * Cameras port forwarding over USB
    for (int port = 5800; port <= 5807; port++) PortForwarder.add(port, Constants.Vision.Limelight3.kName + ".local", port);
    for (int port = 5800; port <= 5807; port++) PortForwarder.add(port, Constants.Vision.Limelight3G.kName + ".local", port);
    for (int port = 5800; port <= 5807; port++) PortForwarder.add(port, Constants.Vision.LimelightTwoPlus.kName + ".local", port);
    for (int port = 5800; port <= 5807; port++) PortForwarder.add(port, "photonvision.local", port);
  
    // * DataLogManager
    DataLogManager.start();
    DataLogManager.logNetworkTables(true);
    DriverStation.startDataLog(DataLogManager.getLog(), true);

    // * Start REV Logging
    URCL.start();

    // * Verify PDP
    boolean pdpValid = false;
    PowerDistributionVersion pdpVersion = null;
    try {
      pdpVersion = pdp.getVersion();
      pdpValid = true;
    } catch (Exception e) {
      pdpValid = false;
    }

    if (!pdpValid) {
      System.out.print("ERROR: NO PDH FOUND AT CAN ID " + String.valueOf(pdp.getModule()));
      buzzer.playTone(200, 0.075);
      Timer.delay(0.05);
      buzzer.playTone(200, 0.075);
    } else {
      if (pdpVersion != null) System.out.print("PDP Version: " + String.valueOf(pdpVersion.firmwareMajor) + "." + String.valueOf(pdpVersion.firmwareMinor));
      buzzer.playTone(560, 0.075);
      Timer.delay(0.05);
      buzzer.playTone(560, 0.075);
    }

    // * Beep
    buzzer.playTone(440, 0.1);
    Timer.delay(0.1);
    buzzer.playTone(440, 0.1);
    Timer.delay(0.1);
    buzzer.playTone(440, 0.1);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) m_autonomousCommand.schedule();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) m_autonomousCommand.cancel();
  }

  @Override
  public void teleopPeriodic() {}

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
