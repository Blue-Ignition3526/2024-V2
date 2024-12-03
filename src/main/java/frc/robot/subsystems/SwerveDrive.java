package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Gyro.Gyro;
import lib.BlueShift.control.SpeedAlterator;

import static edu.wpi.first.units.Units.MetersPerSecond;

public class SwerveDrive extends SubsystemBase {
    // * Swerve Modules
    SwerveModule frontLeft;
    SwerveModule frontRight;
    SwerveModule backLeft;
    SwerveModule backRight;

    // * Gyro
    Gyro gyro;

    // * Odometry
    SwerveDrivePoseEstimator odometry;

    // * Speed stats
    boolean drivingRobotRelative = false;
    ChassisSpeeds unalteredSpeeds = new ChassisSpeeds();
    ChassisSpeeds desiredSpeeds = new ChassisSpeeds();
    SpeedAlterator speedAlterator = null;

    // * Logging
    public final Sendable targetModulePublisher;
    public final Sendable realModulePublisher;

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

        // Logging
        this.realModulePublisher = new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("SwerveDrive");

                SwerveModuleState frontLeftState = frontLeft.getRealState();
                builder.addDoubleProperty("Front Left Angle", () -> frontLeftState.angle.getRadians(), null);
                builder.addDoubleProperty("Front Left Velocity", () -> frontLeftState.speedMetersPerSecond, null);

                SwerveModuleState frontRightState = frontRight.getRealState();
                builder.addDoubleProperty("Front Right Angle", () -> frontRightState.angle.getRadians(), null);
                builder.addDoubleProperty("Front Right Velocity", () -> frontRightState.speedMetersPerSecond, null);

                SwerveModuleState backLeftState = backLeft.getRealState();
                builder.addDoubleProperty("Back Left Angle", () -> backLeftState.angle.getRadians(), null);
                builder.addDoubleProperty("Back Left Velocity", () -> backLeftState.speedMetersPerSecond, null);

                SwerveModuleState backRightState = backRight.getRealState();
                builder.addDoubleProperty("Back Right Angle", () -> backRightState.angle.getRadians(), null);
                builder.addDoubleProperty("Back Right Velocity", () -> backRightState.speedMetersPerSecond, null);

                builder.addDoubleProperty("Robot Angle", () -> getHeading().getRadians(), null);
            }
        };

        this.targetModulePublisher = new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("SwerveDrive");

                SwerveModuleState frontLeftState = frontLeft.getTargetState();
                builder.addDoubleProperty("Front Left Angle", () -> frontLeftState.angle.getRadians(), null);
                builder.addDoubleProperty("Front Left Velocity", () -> frontLeftState.speedMetersPerSecond, null);

                SwerveModuleState frontRightState = frontRight.getTargetState();
                builder.addDoubleProperty("Front Right Angle", () -> frontRightState.angle.getRadians(), null);
                builder.addDoubleProperty("Front Right Velocity", () -> frontRightState.speedMetersPerSecond, null);

                SwerveModuleState backLeftState = backLeft.getTargetState();
                builder.addDoubleProperty("Back Left Angle", () -> backLeftState.angle.getRadians(), null);
                builder.addDoubleProperty("Back Left Velocity", () -> backLeftState.speedMetersPerSecond, null);

                SwerveModuleState backRightState = backRight.getTargetState();
                builder.addDoubleProperty("Back Right Angle", () -> backRightState.angle.getRadians(), null);
                builder.addDoubleProperty("Back Right Velocity", () -> backRightState.speedMetersPerSecond, null);

                builder.addDoubleProperty("Robot Angle", () -> getHeading().getRadians(), null);
            }
        };
    }

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
     * Reset the pose of the robot to (0, 0)
     */
    public void resetPose() {
        resetOdometry(new Pose2d());
    }

    /**
     * Reset the pose of the robot to the provided pose
     * @param pose
     */
    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(this.getHeading(), getModulePositions(), pose);
    }

    public ChassisSpeeds getRobotRelativeChassisSpeeds() {
        if (this.drivingRobotRelative) return this.unalteredSpeeds;
        else return ChassisSpeeds.fromFieldRelativeSpeeds(unalteredSpeeds, getHeading());
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
        this.unalteredSpeeds = speeds;
        ChassisSpeeds desiredSpeeds = this.speedAlterator != null ? this.speedAlterator.alterSpeed(speeds, drivingRobotRelative) : speeds;
        this.desiredSpeeds = desiredSpeeds;
        SwerveModuleState[] m_moduleStates = Constants.SwerveDrive.PhysicalModel.kDriveKinematics.toSwerveModuleStates(desiredSpeeds);
        this.setModuleStates(m_moduleStates);
    }

    public Command enableSpeedAlteratorCommand(SpeedAlterator alterator) {
        return runOnce(() -> this.speedAlterator = alterator);
    }

    public Command disableSpeedAlteratorCommand() {
        return runOnce(() -> this.speedAlterator = null);
    }

    /**
     * Drive the robot with the provided speeds <b>(ROBOT RELATIVE)</b>
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
        // Log data
        SmartDashboard.putNumber("SwerveDrive/RobotHeadingRad", this.getHeading().getRadians());
        SmartDashboard.putNumber("SwerveDrive/RobotHeadingDeg", this.getHeading().getDegrees());
        SmartDashboard.putBoolean("SwerveDrive/RobotRelative", this.drivingRobotRelative);
        SmartDashboard.putData("SwerveDrive/RealModuleStates", realModulePublisher);
        SmartDashboard.putData("SwerveDrive/TargetModuleStates", targetModulePublisher);
    }
}
