package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.fasterxml.jackson.databind.PropertyNamingStrategies.UpperCamelCaseStrategy;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import lib.BlueShift.constants.CTRECANDevice;
import lib.BlueShift.constants.PIDFConstants;
import lib.BlueShift.constants.SwerveModuleOptions;
import lib.BlueShift.math.InterpolatingTable;
import lib.BlueShift.utils.SwerveChassis;

public final class Constants {

  public static class Elevator {
    //Motors Id
    public static final int kleftElevatorMotorId = 2;
    public static final int krightElevatorMotorId = 3;

    //Values of Concersions
    public static final double kradiansToInches = 1.897; 

    //Motion (PID and Constraints)
    // TODO: add the correct maxVelocity and maxAcceleration
    public static final Constraints kElevatorConstraints = new Constraints(26, 35);

    public static final double kElevatorP = 0.1;
    public static final double kElevatorI = 0.0;
    public static final double kElevatorD = 0.0;

    //Bounds in inches
    public static final double kupperBound = 8.0;
    public static final double kmediumBound = 5.0;
    public static final double klowerBound = 0.0;

    public enum ElevatorPosition {
      HIGH("High", 8.0),
      MEDIUM("Medium", 5.0 ),
      LOW("Low", 0.0);

      private final String name;
      private final double position;

      ElevatorPosition(String name, double position){
        this.name = name;
        this.position = position;
      }

      public String getName(){
        return name;
      }

      public double getPosition(){
        return position;
      }
    }

    //Speeds
    public static final double kspeedUp = 0.3;
    public static final double kspeedDown = -0.3;
    public static final double kspeedInPlace = 0.05;
  }
  public static class Intake {
    // * Speeds
    public static final double kInSpeed = 0.8d;
    public static final double kOutSpeed = -0.3d;
    public static final double kAvoidSpeed = -0.05d;

    // * Motor
    public static final int kMotorId = 5;
    public static final int kMotorMaxCurrent = 60;
  }

  // * Shooter
  // TODO: subir limit (listo creo)
  // TODO: Check speeds
  // TODO: Check angles
  // TODO: Set motor IDs
  
  public static final class ShooterSubsystem {
    public static final int kUpperRollerID = 1;
    public static final int kLowerRollerID = 2;
    public static final double upperSpeed = 0;
    public static final double lowerSpeed = 0;
    public static final double finalLowerSpeed = 0;
    public static final double finalUpperSpeed = 0;

    public static final Measure<Velocity<Angle>> kShooterIdleSpeed = RPM.of(10);

    public static final InterpolatingTable kShooterSpeed = new InterpolatingTable(new double[][] {
      // <distance (meters)>, <speed (RPM)>
      {0, 80},
      {2, 160},
    });

    public static final InterpolatingTable kIndexerAngle = new InterpolatingTable(new double[][] {
      // <distance (meters)>, <angle (degrees)>
      {0, 0},
      {2, 15},
    });
    public static final int UpperSmartCurrentLimit = 40; // voltage limit of upper roller
    public static final double UpperClosedLoopRampRate  = 0.15; // velocity from 0 to 100 upprt roller
    public static final int LowerSmartCurrentLimit = 40; //voltage limit from lower roller
    public static final double lowerClosedLoopRampRate = 0.15; // velocity from 0 to 100 lower roller

    public static final double kVelocityToleranceRPM = 10;
  }

  // indexer
  public static final class Indexer {
    public static final class Pivot {
      // * Encoder
      // TODO: Check angles
      public static final int kIndexerPivotEncoderPort = 1;
      public static final Measure<Angle> kIndexerPivotEncoderOffset = Degrees.of(0);
      // public static final Measure<Angle> kIndexerPivotMinAngle = Degrees.of(0);
      // public static final Measure<Angle> kIndexerPivotMaxAngle = Degrees.of(0);
  
      // * Motor
      // TODO: Set motor ID
      public static final int kIndexerPivotMotorId = 8;
      public static final int kIndexerPivotMotorMaxCurrent = 20;
  
      // * Control
      // TODO: Check values
      public static final TrapezoidProfile.Constraints kIndexerPivotConstraints = new TrapezoidProfile.Constraints(500, 500);
      public static final ProfiledPIDController kIndexerPivotPIDController = new ProfiledPIDController(0.15, 0, 0, kIndexerPivotConstraints);
      public static final Measure<Angle> kIndexerPivotTolerance = Degrees.of(1);
    }
  }

  public static final class BreamBreaks {
    public static final int kIntakeBeamBreakPort = 1;
    public static final int kIndexerStage1BeamBreakPort = 2;
    public static final int kIndexerStage2BeamBreakPort = 3;
  }

  public static final class Vision {
    public static final class Limelight3G {
      public static final String kName = "limelight-threeg";
      public static final int kOdometryPipeline = 0;
      public static final int kSpeakerPipeline = 1;
      public static final int kViewfinderPipeline = 2;
    }

    public static final class Limelight3 {
      public static final String kName = "limelight-three";
      public static final int kNotePipeline = 0;
      public static final int kOdometryPipeline = 1;
      public static final int kViewfinderPipeline = 2;
    }

    public static final class LimelightTwoPlus {
      public static final String kName = "limelight-twoplus";
      public static final int kOdometryPipeline = 0;
      public static final int kViewfinderPipeline = 1;
    }

    public static final class Arducam_Left {
      public static final String kName = "Arducam_Left";
      public static final Transform3d kRobotToCamera = new Transform3d(-0.61d, 0d, 0.6d, new Rotation3d(0, 0, 5d));
    }

    public static final class Arducam_Right {
      public static final String kName = "Arducam_Right";
      public static final Transform3d kRobotToCamera = new Transform3d(0.61d, 0d, 0.6d, new Rotation3d(0, 0, -5d));
    }
  }

  public static final class Field {
    public static final Pose2d kInitialPoseMeters = new Pose2d(new Translation2d(1, 2), new Rotation2d(0, 0));
    public static final Pose3d kBlueSpeakerPoseMeters = new Pose3d(new Translation3d(0, 5.55, 2), new Rotation3d(0, 0, 0));
    public static final Pose3d kRedSpeakerPoseMeters = new Pose3d(new Translation3d(17, 5.55, 2), new Rotation3d(0, 0, 0));
  }

  public static final class SwerveDrive {
        // * Gyro
        public static final CTRECANDevice kGyroDevice = new CTRECANDevice(34, "*");

        // * Controller
        public static final double kJoystickDeadband = 0.1;

        // * Heading Controller
        public static final Measure<Angle> kHeadingTolerance = Degrees.of(3);
        public static final TrapezoidProfile.Constraints kHeadingConstraints = new TrapezoidProfile.Constraints(100, 10);
        public static final ProfiledPIDController kHeadingController = new ProfiledPIDController(0.1, 0, 0, kHeadingConstraints);

        // * Physical model of the robot
        public static final class PhysicalModel {
            // * MAX DISPLACEMENT SPEED (and acceleration)
            public static Measure<Velocity<Distance>> kMaxSpeed = MetersPerSecond.of(4);
            public static final Measure<Velocity<Velocity<Distance>>> kMaxAcceleration = MetersPerSecondPerSecond.of(3);

            // * MAX ROTATIONAL SPEED (and acceleration)
            public static final Measure<Velocity<Angle>> kMaxAngularSpeed = RotationsPerSecond.of(1);
            public static final Measure<Velocity<Velocity<Angle>>> kMaxAngularAcceleration = RotationsPerSecond.per(Second).of(Math.pow(kMaxAngularSpeed.in(RotationsPerSecond), 2));

            // * Drive wheel diameter
            public static final Measure<Distance> kWheelDiameter = Inches.of(4);

            // * Gear ratios
            public static final double kDriveMotorGearRatio = 1.0 / 6.75;
            public static final double kTurningMotorGearRatio = 1.0 / 21.428571428571427;

            // * Conversion factors (Drive Motor) DO NOT CHANGE
            public static final double kDriveEncoder_RotationToMeter = kDriveMotorGearRatio * (kWheelDiameter.in(Meters) / 2) * 2 * Math.PI;
            public static final double kDriveEncoder_RPMToMeterPerSecond = kDriveEncoder_RotationToMeter / 60.0;

            // * Conversion factors (Turning Motor) DO NOT CHANGE
            public static final double kTurningEncoder_RotationToRadian = kTurningMotorGearRatio * 2.0 * Math.PI;
            public static final double kTurningEncoder_RPMToRadianPerSecond = kTurningEncoder_RotationToRadian / 60.0;

            // * Robot Without bumpers measures
            public static final Measure<Distance> kTrackWidth = Inches.of(26);
            public static final Measure<Distance> kWheelBase = Inches.of(30.5);
    
            // * Create a kinematics instance with the positions of the swerve modules
            public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(SwerveChassis.sizeToModulePositions(kTrackWidth.in(Meters), kWheelBase.in(Meters)));
        }

        // * Swerve modules configuration
        public static final class SwerveModules {
            // * PID
            public static final PIDFConstants kTurningPIDConstants = new PIDFConstants(0.5);

            // * Swerve modules options
            public static final SwerveModuleOptions kFrontLeftOptions = new SwerveModuleOptions()
                .setAbsoluteEncoderInverted(false)
                .setAbsoluteEncoderCANDevice(new CTRECANDevice(11, "*"))
                .setDriveMotorID(22)
                .setTurningMotorID(21)
                .setTurningMotorInverted(true)
                .setName("Front Left");

            public static final SwerveModuleOptions kFrontRightOptions = new SwerveModuleOptions()
                .setAbsoluteEncoderInverted(false)
                .setAbsoluteEncoderCANDevice(new CTRECANDevice(12, "*"))
                .setDriveMotorID(24)
                .setTurningMotorID(23)
                .setTurningMotorInverted(true)
                .setName("Front Right");

            public static final SwerveModuleOptions kBackLeftOptions = new SwerveModuleOptions()
                .setAbsoluteEncoderInverted(false)
                .setAbsoluteEncoderCANDevice(new CTRECANDevice(13, "*"))
                .setDriveMotorID(26)
                .setTurningMotorID(25)
                .setTurningMotorInverted(true)
                .setName("Back Left");


            public static final SwerveModuleOptions kBackRightOptions = new SwerveModuleOptions()
                .setAbsoluteEncoderInverted(false)
                .setAbsoluteEncoderCANDevice(new CTRECANDevice(14, "*"))
                .setDriveMotorID(28)
                .setTurningMotorID(27)
                .setTurningMotorInverted(true)
                .setName("Back Right");
        }
    }
}
