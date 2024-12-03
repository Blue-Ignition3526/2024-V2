package frc.robot;

import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.commands.ClimberCommands;
import frc.robot.commands.SwerveDrive.DriveSwerve;
import frc.robot.subsystems.BeamBreaks;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.IndexerPivot;
import frc.robot.subsystems.IndexerRollers;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.Elevator.ElevatorPosition;
import frc.robot.subsystems.Gyro.Gyro;
import frc.robot.subsystems.Gyro.GyroIOPigeon;
import frc.robot.subsystems.IndexerPivot.IndexerPivotPosition;
import lib.BlueShift.control.CustomController;
import lib.BlueShift.control.CustomController.CustomControllerType;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

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
  
  // * BeamBreaks
  private final BeamBreaks m_beamBreaks;
  
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

  //* Climbers
  // private final Climber m_leftClimber;
  // private final Climber m_rightClimber;

  // * Vision
  SwerveDrivePoseEstimator m_poseEstimator;

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

    // BeamBreaks
    this.m_beamBreaks = new BeamBreaks();

    // Intake
    this.m_intake = new Intake();

    // Elevator
    this.m_elevator = new Elevator();

    // Shooter
    this.m_shooter = new Shooter();

    // Indexer Pivot
    this.m_indexerPivot = new IndexerPivot();

    // Indexer Rollers
    this.m_indexerRollers = new IndexerRollers(m_beamBreaks);

    // Climbers
    // this.m_leftClimber = new Climber("LeftClimber", Constants.Climber.kLeftClimberMotorID);
    // this.m_rightClimber = new Climber("RightClimber", Constants.Climber.kRightClimberMotorID);

    // Vision
    LimelightHelpers.setPipelineIndex(Constants.Vision.Limelight3G.kName, Constants.Vision.Limelight3G.kOdometryPipeline);
    m_poseEstimator = new SwerveDrivePoseEstimator(
      Constants.SwerveDrive.PhysicalModel.kDriveKinematics,
      m_swerveDrive.getHeading(),
      m_swerveDrive.getModulePositions(),
      new Pose2d()
    );

    // Dashboard Commands
    // Drivetrain
    SmartDashboard.putData("Commands/Drivetrain/DrivetrainZeroHeading", new InstantCommand(m_swerveDrive::zeroHeading));
    SmartDashboard.putData("Commands/Drivetrain/DrivetrainResetEncoders", new InstantCommand(m_swerveDrive::resetTurningEncoders));

    // BeamBreaks
    SmartDashboard.putData("Commands/Drivetrain/EnableBeamBreaks", new InstantCommand(m_beamBreaks::enable));
    SmartDashboard.putData("Commands/Drivetrain/DisableBeamBreaks", new InstantCommand(m_beamBreaks::disable));

    // Climbers
    // SmartDashboard.putData("Commands/Climbers/SetClimbersLow", ClimberCommands.setClimbersLowCommand(m_leftClimber, m_rightClimber));
    // SmartDashboard.putData("Commands/Climbers/SetClimbersHigh", ClimberCommands.setClimbersHighCommand(m_leftClimber, m_rightClimber));

    // Elevator
    SmartDashboard.putData("Commands/Elevator/SetElevatorLow", m_elevator.setPositionCommand(ElevatorPosition.LOW));
    SmartDashboard.putData("Commands/Elevator/SetElevatorMid", m_elevator.setPositionCommand(ElevatorPosition.MEDIUM));
    SmartDashboard.putData("Commands/Elevator/SetElevatorHigh", m_elevator.setPositionCommand(ElevatorPosition.HIGH));

    // IndexerPivot
    SmartDashboard.putNumber("Commands/IndexerPivot/IndexerSetpoint", 0);
    SmartDashboard.putData("Commands/IndexerPivot/SetSetpoint", m_indexerPivot.setSetpointCommand(Degrees.of(SmartDashboard.getNumber("Commands/IndexerPivot/IndexerSetpoint", 0))));

    // IndexerRollers
    SmartDashboard.putData("Commands/IndexerRollers/IndexerRollersIn", m_indexerRollers.setRollersInCommand());
    SmartDashboard.putData("Commands/IndexerRollers/IndexerRollersOut", m_indexerRollers.setRollersOutCommand());
    SmartDashboard.putData("Commands/IndexerRollers/IndexerRollersStop", m_indexerRollers.stopRollersCommand());

    // Intake
    SmartDashboard.putData("Commands/Intake/IntakeIn", m_intake.setInCommand());
    SmartDashboard.putData("Commands/Intake/IntakeOut", m_intake.setOutCommand());
    SmartDashboard.putData("Commands/Intake/IntakeAvoid", m_intake.setAvoidCommand());

    // Shooter
    // SmartDashboard.putNumber("Commands/Shooter/ShooterSpeed", 140);
    // SmartDashboard.putData("Commands/Shooter/SetSetpoint", m_shooter.setRpmCommand(RPM.of(SmartDashboard.getNumber("Commands/Shooter/ShooterSpeed", 140))));

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
    
    // * Default control
    this.m_controller.bottomButton().toggleOnTrue(new SequentialCommandGroup(
      new ParallelCommandGroup(
        m_elevator.setPositionCommand(ElevatorPosition.LOW),
        m_indexerPivot.setSetpointCommand(IndexerPivotPosition.RECEIVING)
      ),
      m_intake.setInCommand(),
      m_indexerRollers.receiveCommand()
    ));

    this.m_controller.topButton().onTrue(new SequentialCommandGroup(
      new ParallelCommandGroup(
        m_elevator.setPositionCommand(ElevatorPosition.LOW),
        m_indexerPivot.setSetpointCommand(IndexerPivotPosition.RECEIVING)
      ),
      m_intake.setOutCommand(),
      m_indexerRollers.setRollersOutCommand(),
      new WaitCommand(0.25),
      m_indexerRollers.stopRollersCommand()
    ));

    this.m_controller.leftTrigger().whileTrue(new ParallelCommandGroup(
      m_elevator.setPositionCommand(ElevatorPosition.LOW),
      m_indexerPivot.setSetpointCommand(
        Degrees.of(
          Constants.Shooter.kIndexerAngle.getInterpolatedValue(
            Constants.Field.getCurrentSpeaker().toPose2d().minus(m_poseEstimator.getEstimatedPosition()).getTranslation().getNorm()
          )
        )
      ),
      m_shooter.setRpmCommand(RPM.of(
        Constants.Shooter.kShooterSpeed.getInterpolatedValue(
            Constants.Field.getCurrentSpeaker().toPose2d().minus(m_poseEstimator.getEstimatedPosition()).getTranslation().getNorm()
          )
        )
      )
    ));

    this.m_controller.rightTrigger().whileTrue(m_indexerRollers.setRollersPassCommand());
    
    // this.m_controller.bottomButton().toggleOnTrue(m_intake.setInCommand());

    // * Intake
    this.m_intake.setDefaultCommand(m_intake.setAvoidCommand());
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

  public void periodic() {
    // Update odometry
    m_poseEstimator.update(m_swerveDrive.getHeading(), m_swerveDrive.getModulePositions());

    boolean doRejectUpdate = false;
    LimelightHelpers.SetRobotOrientation(Constants.Vision.Limelight3G.kName, m_gyro.getHeading().getDegrees(), m_gyro.getYawVelocity(), 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
    if(Math.abs(m_gyro.getYawVelocity()) > 720 || !m_elevator.getSetpoint().equals(ElevatorPosition.LOW)) {
      doRejectUpdate = true;
    }
    if(mt2.tagCount == 0) {
      doRejectUpdate = true;
    }
    if(!doRejectUpdate) {
      m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
      m_poseEstimator.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
    }
  }
}



