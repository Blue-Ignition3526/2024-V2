package frc.robot;

import frc.robot.commands.Intake.IntakeIn;
import frc.robot.commands.SwerveDrive.DriveSwerve;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.Gyro.Gyro;
import frc.robot.subsystems.Gyro.GyroIOPigeon;
import lib.BlueShift.control.CustomController;
import lib.BlueShift.control.CustomController.CustomControllerType;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {
  // * Controller
  private final CustomController m_controller = new CustomController(0, CustomControllerType.XBOX);
  
  // * Gyro
  private final Gyro m_gyro;
  
  // * Swerve Modules
  private final SwerveModule m_frontLeft;
  private final SwerveModule m_frontRight;
  private final SwerveModule m_backLeft;
  private final SwerveModule m_backRight;
  
  // * Swerve Drive
  private final SwerveDrive m_swerveDrive;
  
  // * Intake
  private final Intake m_intake;

  public RobotContainer() {
    // Gyro
    this.m_gyro = new Gyro(new GyroIOPigeon(Constants.SwerveDrive.kGyroDevice));

    // Swerve Modules
    this.m_frontLeft = new SwerveModule(Constants.SwerveDrive.SwerveModules.kFrontLeftOptions);
    this.m_frontRight = new SwerveModule(Constants.SwerveDrive.SwerveModules.kFrontRightOptions);
    this.m_backLeft = new SwerveModule(Constants.SwerveDrive.SwerveModules.kBackLeftOptions);
    this.m_backRight = new SwerveModule(Constants.SwerveDrive.SwerveModules.kBackRightOptions);

    // Swerve Drive
    this.m_swerveDrive = new SwerveDrive(m_frontLeft, m_frontRight, m_backLeft, m_backRight, m_gyro);

    // Intake
    this.m_intake = new Intake();

    configureBindings();
  }

  private void configureBindings() {
    this.m_swerveDrive.setDefaultCommand(new DriveSwerve(
        m_swerveDrive,
        () -> -m_controller.getLeftY(),
        () -> -m_controller.getLeftX(),
        () -> -m_controller.getRightX(),
        () -> true,
        () -> false
      )
    );

    this.m_intake.setDefaultCommand(new IntakeIn(m_intake));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
