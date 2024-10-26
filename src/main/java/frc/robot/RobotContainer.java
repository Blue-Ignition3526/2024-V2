package frc.robot;

import frc.robot.commands.SwerveDrive.DriveSwerve;
import frc.robot.subsystems.Intake;
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

  // shooter
  // private final ShooterSubsystem m_ShooterSubsystem;


  // * Indexer Pivot
  // private final IndexerPivot m_indexerPivot;

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

    SmartDashboard.putData("ZeroHeading", new InstantCommand(() -> m_swerveDrive.zeroHeading()));
    SmartDashboard.putData("ResetTurningEncoders", new InstantCommand(() -> m_swerveDrive.resetTurningEncoders()));

    //shooter
    // this.m_ShooterSubsystem = new ShooterSubsystem();


    // Indexer Pivot
    // this.m_indexerPivot = new IndexerPivot();

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
  }

  public Command getAutonomousCommand() {
    // return Commands.repeatingSequence(
    //   new SetIndexerPivotAngle(m_indexerPivot, Degrees.of(0)),
    //   Commands.waitSeconds(0.5),
    //   new SetIndexerPivotAngle(m_indexerPivot, Degrees.of(-90)),
    //   Commands.waitSeconds(0.5),
    //   new SetIndexerPivotAngle(m_indexerPivot, Degrees.of(90)),
    //   Commands.waitSeconds(0.5),
    //   new SetIndexerPivotAngle(m_indexerPivot, Degrees.of(0)),
    //   Commands.waitSeconds(0.5)
    // );
    return null;
  }
}
