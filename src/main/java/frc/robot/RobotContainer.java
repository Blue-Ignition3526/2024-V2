package frc.robot;

import frc.robot.commands.CompoundCommands;
import frc.robot.commands.SwerveDrive.DriveSwerve;
import frc.robot.speedAlterators.AlignToSpeaker;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.IndexerPivot;
import frc.robot.subsystems.IndexerRollers;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Odometry;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.Gyro.Gyro;
import frc.robot.subsystems.Gyro.GyroIOPigeon;
import lib.BlueShift.control.CustomController;
import lib.BlueShift.control.SpeedAlterator;
import lib.BlueShift.control.CustomController.CustomControllerType;
import lib.BlueShift.odometry.vision.VisionOdometryPoseEstimate;
import lib.BlueShift.odometry.vision.camera.LimelightOdometryCamera;
import lib.BlueShift.odometry.vision.camera.VisionOdometryFilters;

import java.util.Optional;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

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
  private final SpeedAlterator alignToSpeakerAlterator;
  
  // * Intake
  private final Intake m_intake;

  // * Elevator
  private final Elevator m_elevator;

  // * Shooter
  private final Shooter m_shooter;

  // * Indexer Pivot
  private final IndexerPivot m_indexerPivot;

  // * Indexer Rollers
  private final IndexerRollers m_indexerRollers;

  // * Odometry
  LimelightOdometryCamera limelight3G;
  LimelightOdometryCamera limelight3;
  Odometry m_odometry;

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

    // Elevator
    this.m_elevator = new Elevator();

    // Shooter
    this.m_shooter = new Shooter();

    // Indexer Pivot
    this.m_indexerPivot = new IndexerPivot();

    // Indexer Rollers
    this.m_indexerRollers = new IndexerRollers();

    // Vision
    LimelightHelpers.setPipelineIndex(Constants.Vision.Limelight3G.kName, Constants.Vision.Limelight3G.kOdometryPipeline);
    LimelightHelpers.setPipelineIndex(Constants.Vision.Limelight3.kName, Constants.Vision.Limelight3.kOdometryPipeline);

    this.limelight3G = new LimelightOdometryCamera(Constants.Vision.Limelight3G.kName, false, VisionOdometryFilters::limelightFilter);
    this.limelight3 = new LimelightOdometryCamera(Constants.Vision.Limelight3.kName, false, VisionOdometryFilters::limelightFilter);

    this.m_odometry = new Odometry(
      Constants.SwerveDrive.PhysicalModel.kDriveKinematics,
      m_swerveDrive::getHeading,
      m_swerveDrive::getModulePositions,
      new Pose2d(),
      0.02,
      true
    );

    this.m_odometry.addCamera(limelight3G);
    this.m_odometry.addCamera(limelight3);

    this.limelight3G.enable();
    this.limelight3.enable();

    this.m_odometry.setVisionEnabled(true);
    this.m_odometry.startVisionLoop();

    // Speed Alterators
    this.alignToSpeakerAlterator = new AlignToSpeaker(m_odometry::getPose);

    // Dashboard Commands
    // Drivetrain
    SmartDashboard.putData("Commands/Drivetrain/DrivetrainZeroHeading", new InstantCommand(m_swerveDrive::zeroHeading));
    SmartDashboard.putData("Commands/Drivetrain/DrivetrainResetEncoders", new InstantCommand(m_swerveDrive::resetTurningEncoders));

    // IndexerPivot
    SmartDashboard.putNumber("Commands/IndexerPivot/IndexerSetpoint", 0);

    // IndexerRollers
    SmartDashboard.putData("Commands/IndexerRollers/IndexerRollersIn", m_indexerRollers.setRollersInCommand());
    SmartDashboard.putData("Commands/IndexerRollers/IndexerRollersOut", m_indexerRollers.setRollersOutCommand());
    SmartDashboard.putData("Commands/IndexerRollers/IndexerRollersStop", m_indexerRollers.stopRollersCommand());

    // Intake
    SmartDashboard.putData("Commands/Intake/IntakeIn", m_intake.setInCommand());
    SmartDashboard.putData("Commands/Intake/IntakeOut", m_intake.setOutCommand());
    SmartDashboard.putData("Commands/Intake/IntakeAvoid", m_intake.setAvoidCommand());
    
    SmartDashboard.putData("Commands/Odometry/SetVisionPose", new InstantCommand(this::setVisionPose));

    configureBindings();
  }

  private void configureBindings() {
    // * Drivetrain
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

    this.m_controller.rightBumper().onTrue(m_swerveDrive.enableSpeedAlteratorCommand(alignToSpeakerAlterator));
    this.m_controller.rightBumper().onFalse(m_swerveDrive.disableSpeedAlteratorCommand());
    
    // * Intake command
    this.m_controller.bottomButton().toggleOnTrue(CompoundCommands.intakeCommand(
      m_indexerPivot,
      m_elevator,
      m_shooter,
      m_intake,
      m_indexerRollers,
      m_indexerRollers::getBeamBreak
    ));

    // * DISABLE BEAM BREAK MOMENTARILY COMMAND
    this.m_controller.leftBumper()
    .onTrue(m_indexerRollers.setBeamBreakEnabledCommand(false))
    .onFalse(m_indexerRollers.setBeamBreakEnabledCommand(true));

    // * Outtake command
    this.m_controller.topButton().whileTrue(CompoundCommands.outtakeCommand(
      m_indexerPivot,
      m_elevator,
      m_shooter,
      m_intake,
      m_indexerRollers
    ));

    // * High shoot command
    this.m_controller.rightTrigger().whileTrue(CompoundCommands.highShootCommand(
      m_indexerPivot,
      m_indexerRollers,
      m_shooter
    ));

    // * Low shoot command
    this.m_controller.leftTrigger().whileTrue(CompoundCommands.lowShootCommand(
      m_indexerPivot,
      m_indexerRollers,
      m_shooter
    ));

    // * Elevator high position command
    this.m_controller.povUp().onTrue(m_elevator.setPositionCommand(55));

    // * Elevator low position command
    this.m_controller.povDown().onTrue(m_elevator.setPositionCommand(0));

    // * Amp command
    this.m_controller.povRight().whileTrue(CompoundCommands.ampCommand(m_indexerPivot, m_indexerRollers, m_elevator));

    // * Indexer out command
    this.m_controller.leftButton().onTrue(m_indexerRollers.setRollersOutCommand()).onFalse(m_indexerRollers.stopRollersCommand());
  }

  public Command getAutonomousCommand() {
    return null;
  }

  public Command getTeleopCommand() {
    return new ParallelCommandGroup(
      m_elevator.resetPIDCommand(),
      m_indexerPivot.resetPIDCommand()
    );
  }

  public void setVisionPose() {
    Optional<VisionOdometryPoseEstimate> poseEstimate = limelight3G.getEstimate();
    if (poseEstimate.isPresent() && limelight3G.isEnabled()) m_odometry.resetPose(poseEstimate.get().pose);
  }

  public void autonomousInit() {
    setVisionPose();
  }
  public void autonomousPeriodic() {}

  public void teleopInit() {
    setVisionPose();
  }
  public void teleopPeriodic() {}

  public void init() {}
  public void periodic() {
    // Update limelights' gyroscope values
    LimelightHelpers.SetRobotOrientation(Constants.Vision.Limelight3G.kName, m_gyro.getHeading().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation(Constants.Vision.Limelight3.kName, m_gyro.getHeading().getDegrees(), 0, 0, 0, 0, 0);
  }
}



