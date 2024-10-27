package frc.robot;

import frc.robot.Constants.Elevator;
import frc.robot.commands.SwerveDrive.DriveSwerve;
import frc.robot.subsystems.BeamBreaks;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.IndexerPivot;
import frc.robot.subsystems.IndexerRollers;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.Gyro.Gyro;
import frc.robot.subsystems.Gyro.GyroIOPigeon;
import lib.BlueShift.control.CustomController;
import lib.BlueShift.control.CustomController.CustomControllerType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

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

  // * Shooter
  // private final Shooter m_shooter;

  // * Indexer Pivot
  // private final IndexerPivot m_indexerPivot;

  // * BeamBreaks
  private final BeamBreaks m_beamBreaks;

  //* Climber 
  private final Climber m_climber;
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

    // Shooter
    // this.m_shooter = new Shooter();

    // Indexer Pivot
    // this.m_indexerPivot = new IndexerPivot();

    // BeamBreaks
    this.m_beamBreaks = new BeamBreaks();

    // Climber
    this.m_climber = new Climber();

    SmartDashboard.putData("ZeroHeading", new InstantCommand(() -> m_swerveDrive.zeroHeading()));
    SmartDashboard.putData("ResetTurningEncoders", new InstantCommand(() -> m_swerveDrive.resetTurningEncoders()));

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

    this.m_controller.rightStickButton().onTrue(new InstantCommand(() -> m_swerveDrive.zeroHeading()));
    
    this.m_controller.bottomButton().toggleOnTrue(m_intake.setIn());
    this.m_controller.rightButton().whileTrue(m_intake.setOut());
    this.m_intake.setDefaultCommand(m_intake.setAvoid());

    this.m_controller.leftButton().onTrue(m_climber.setClimberPositionCommand(15));
    this.m_controller.leftButton().onFalse(m_climber.setClimberPositionCommand(0));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}



