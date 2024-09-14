package frc.robot;

import frc.robot.Constants.OperatorConstants;
import lib.BlueShift.control.CustomController;
import lib.BlueShift.control.CustomController.CustomControllerType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  private final CustomController m_controller;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    this.m_controller = new CustomController(Constants.OperatorConstants.kDriverControllerPort, CustomControllerType.XBOX);
    configureBindings();
  }

  private void configureBindings() {
    this.m_controller.leftBumper();
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
