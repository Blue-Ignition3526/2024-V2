package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Gyro.Gyro;
import static edu.wpi.first.units.Units.MetersPerSecond;

public class SwerveDrive extends SubsystemBase {
    // * Swerve Modules
    SwerveModule frontLeft;
    SwerveModule frontRight;
    SwerveModule backLeft;
    SwerveModule backRight;

    // * Gyro
    Gyro gyro;

    // * Speed stats
    boolean drivingRobotRelative = false;
    ChassisSpeeds speeds = new ChassisSpeeds();

    // * Odometry field
    Field2d m_field = new Field2d();

    public SwerveDrive(SwerveModule frontLeft, SwerveModule frontRight, SwerveModule backLeft, SwerveModule backRight, Gyro gyro) {
        // Swerve Modules
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;

        // Gyro
        this.gyro = gyro;
        
        // Reset gyro
        this.gyro.reset();

        // Configure Auto Builder
        // this.configureAutoBuilder();
    }

    /**
     * Configure the auto builder for PathPlanner
     */
    // public void configureAutoBuilder() {
    //     AutoBuilder.configureHolonomic(
    //         this::getPose,
    //         this::resetOdometry,
    //         this::getRobotRelativeChassisSpeeds,
    //         this::driveRobotRelative,
    //         new HolonomicPathFollowerConfig(
    //             Constants.SwerveDrive.Autonomous.kTranslatePIDConstants,
    //             Constants.SwerveDrive.Autonomous.kRotatePIDConstants,
    //             Constants.SwerveDrive.Autonomous.kMaxSpeedMetersPerSecond.in(MetersPerSecond),
    //             Constants.SwerveDrive.PhysicalModel.kWheelBase.in(Meters) / 2,
    //             new ReplanningConfig(true, true)
    //         ),
    //         () -> {
    //             if (DriverStation.getAlliance().isPresent()) return DriverStation.getAlliance().get() == Alliance.Red;
    //             return false;
    //         },
    //         this
    //     );
    // }

    /**
     * Get the current heading of the robot
     */
    public Rotation2d getHeading() {
        return gyro.getHeading();
    }

    /**
     * Zero the heading of the robot
     */
    public void zeroHeading() {
        this.gyro.reset();
    }

    /**
     * Get the current pose of the robot
     * @return
     */
    // public Pose2d getPose() {
    //     return odometry.getEstimatedPosition();
    // }

    /**
     * Reset the pose of the robot to (0, 0)
     */
    // public void resetPose() {
    //     resetOdometry(new Pose2d());
    // }

    /**
     * Reset the pose of the robot to the provided pose
     * @param pose
     */
    // public void resetOdometry(Pose2d pose) {
    //     odometry.resetPosition(this.getHeading(), getModulePositions(), pose);
    // }

    public ChassisSpeeds getRobotRelativeChassisSpeeds() {
        if (this.drivingRobotRelative) return this.speeds;
        else return ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getHeading());
    }

    /**
     * Get the target module states
     * @return
     */
    public SwerveModuleState[] getModuleTargetStates() {
        return new SwerveModuleState[]{
            frontLeft.getTargetState(),
            frontRight.getTargetState(),
            backLeft.getTargetState(),
            backRight.getTargetState()
        };
    }

    /**
     * Get the real module states
     * @return
     */
    public SwerveModuleState[] getModuleRealStates() {
        return new SwerveModuleState[]{
            frontLeft.getRealState(),
            frontRight.getRealState(),
            backLeft.getRealState(),
            backRight.getRealState()
        };
    }

    /**
     * Get the current module positions
     * @return
     */
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[]{
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition(),
        };
    }

   /**
     * Set the module states
     * @param states
     */
    public void setModuleStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.SwerveDrive.PhysicalModel.kMaxSpeed.in(MetersPerSecond));
        frontLeft.setTargetState(states[0]);
        frontRight.setTargetState(states[1]);
        backLeft.setTargetState(states[2]);
        backRight.setTargetState(states[3]);
    }

    /**
     * Drive the robot with the provided speeds <b>(ROBOT RELATIVE)></b>
     * @param xSpeed
     * @param ySpeed
     * @param rotSpeed
     */
    public void drive(ChassisSpeeds speeds) {
        this.speeds = speeds;
        SwerveModuleState[] m_moduleStates = Constants.SwerveDrive.PhysicalModel.kDriveKinematics.toSwerveModuleStates(speeds);
        this.setModuleStates(m_moduleStates);
    }

    /**
     * Drive the robot with the provided speeds <b>(FIELD RELATIVE)</b>
     * @param xSpeed
     * @param ySpeed
     * @param rotSpeed
     */
    public void driveFieldRelative(double xSpeed, double ySpeed, double rotSpeed) {
        this.drivingRobotRelative = false;
        this.drive(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, this.getHeading()));
    }

    /**
     * Drive the robot with the provided speeds <b>(FIELD RELATIVE)</b>
     * @param speeds
     */
    public void driveFieldRelative(ChassisSpeeds speeds) {
        this.drivingRobotRelative = false;
        this.drive(speeds);
    }

    /**
     * Drive the robot with the provided speeds <b>(ROBOT RELATIVE)</b>
     * @param xSpeed
     * @param ySpeed
     * @param rotSpeed
     */
    public void driveRobotRelative(double xSpeed, double ySpeed, double rotSpeed) {
        this.drivingRobotRelative = true;
        this.drive(new ChassisSpeeds(xSpeed, ySpeed, rotSpeed));
    }

    /**
     * Drive the robot with the provided speeds <b>(ROBOT RELATIVE)</b>
     * @param speeds
     */
    public void driveRobotRelative(ChassisSpeeds speeds) {
        this.drivingRobotRelative = true;
        this.drive(speeds);
    }

    /**
     * Stop the robot (sets all motors to 0)
     */
    public void stop() {
        this.frontLeft.stop();
        this.frontRight.stop();
        this.backLeft.stop();
        this.backRight.stop();
    }

    /**
     * Angle all wheels to point inwards in an X pattern
     */
    public void xFormation() {
        this.frontLeft.setTargetState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)), true);
        this.frontRight.setTargetState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)), true);
        this.backLeft.setTargetState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)), true);
        this.backRight.setTargetState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)), true);
    }

    /**
     * Reset the turning encoders of all swerve modules
     */
    public void resetTurningEncoders() {
        this.frontLeft.resetTurningEncoder();
        this.frontRight.resetTurningEncoder();
        this.backLeft.resetTurningEncoder();
        this.backRight.resetTurningEncoder();
    }

    /**
     * Reset the drive encoders of all swerve modules
     */
    public void resetDriveEncoders() {
        this.frontLeft.resetDriveEncoder();
        this.frontRight.resetDriveEncoder();
        this.backLeft.resetDriveEncoder();
        this.backRight.resetDriveEncoder();
    }

    /**
     * Reset all encoders of all swerve modules
     */
    public void resetEncoders() {
        this.resetTurningEncoders();
        this.resetDriveEncoders();
    }

    public void periodic() {
        // Update the field
        // m_field.setRobotPose(this.getPose());

        // Log data
        // SmartDashboard.putData("SwerveDrive/Field", this.m_field);
        // SmartDashboard.putNumber("SwerveDrive/RobotHeadingRad", this.getHeading().getRadians());
        SmartDashboard.putNumber("SwerveDrive/RobotHeadingDeg", this.getHeading().getDegrees());
        SmartDashboard.putBoolean("SwerveDrive/RobotRelative", this.drivingRobotRelative);
        // Logger.recordOutput("SwerveDrive/RobotSpeeds", this.getRobotRelativeChassisSpeeds());
        // Logger.recordOutput("SwerveDrive/ModuleRealStates", this.getModuleRealStates());
        // Logger.recordOutput("SwerveDrive/ModuleTargetStates", this.getModuleTargetStates());
    }
}
