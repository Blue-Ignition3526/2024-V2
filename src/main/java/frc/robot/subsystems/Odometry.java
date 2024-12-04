package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lib.BlueShift.odometry.vision.VisionOdometryPoseEstimate;
import lib.BlueShift.odometry.vision.camera.LimelightOdometryCamera;

public class Odometry extends SubsystemBase {
    private final Field2d m_field = new Field2d();
    private final SwerveDrivePoseEstimator m_poseEstimator;

    // Odometry parameters
    private final SwerveDriveKinematics m_kinematics;
    private final Supplier<Rotation2d> m_gyroAngleSupplier;
    private final Supplier<SwerveModulePosition[]> m_swerveModulePositionsSupplier;
    private final Pose2d m_initialPose;

    // Cameras
    class CameraWorker {
        LimelightOdometryCamera camera;
        Thread thread;

        public CameraWorker(LimelightOdometryCamera camera) {
            this.camera = camera;
        }

        public void setThread(Thread thread) {
            this.thread = thread;
        }

        public Thread getThread() {
            return this.thread;
        }
    }
    private final ArrayList<CameraWorker> m_cameras = new ArrayList<>();

    private final double period;
    private boolean m_visionEnabled;

    public Odometry(
        SwerveDriveKinematics kinematics,
        Supplier<Rotation2d> gyroAngleSupplier,
        Supplier<SwerveModulePosition[]> swerveModulePositionsSupplier,
        Pose2d initialPose,
        double period,
        boolean visionEnabled
    ) {
        m_poseEstimator = new SwerveDrivePoseEstimator(
            kinematics,
            gyroAngleSupplier.get(),
            swerveModulePositionsSupplier.get(),
            initialPose,
            null,
            null
        );

        this.m_kinematics = kinematics;
        this.m_gyroAngleSupplier = gyroAngleSupplier;
        this.m_swerveModulePositionsSupplier = swerveModulePositionsSupplier;
        this.m_initialPose = initialPose;

        this.period = period;

        this.m_visionEnabled = visionEnabled;
    }

    /**
     * Add a camera to the odometry system
     * @see {@link Odometry#start} as you'll have to call it after adding all cameras
     * @see {@link Odometry#stop} to stop the cameras
     * @param camera
     */
    public synchronized void addCamera(LimelightOdometryCamera camera) {
        m_cameras.add(new CameraWorker(camera));
    }

    public synchronized void setVisionEnabled(boolean enabled) {
        m_visionEnabled = enabled;
    }

    public synchronized void startVisionLoop() {
        for (CameraWorker camera : m_cameras) {
            Thread thread = new Thread(() -> {
                while (true) {
                    if (!camera.camera.isEnabled() || !m_visionEnabled) continue;
                    if (camera.camera instanceof LimelightOdometryCamera) ((LimelightOdometryCamera)camera.camera).setHeading(this.m_gyroAngleSupplier.get().getDegrees());
                    Optional<VisionOdometryPoseEstimate> estimate = camera.camera.getEstimate();
                    if (estimate.isEmpty()) continue;
                    m_poseEstimator.addVisionMeasurement(estimate.get().pose, estimate.get().timestamp, estimate.get().stdDev);
                    try {Thread.sleep((long)period * 1000);} catch (InterruptedException e) {break;}
                }
            }, camera.camera.getCameraName() + " Odometry Camera Thread");
            camera.setThread(thread);
            thread.start();
        }
    }

    public synchronized void stopVisionLoop() {
        for (CameraWorker camera : m_cameras) {
            camera.getThread().interrupt();
        }
    }

    public SwerveDrivePoseEstimator getEstimator() {
        return m_poseEstimator;
    }

    public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    public void resetPose(Pose2d pose) {
        m_poseEstimator.resetPosition(m_gyroAngleSupplier.get(), m_swerveModulePositionsSupplier.get(), pose);
    }

    @Override
    public void periodic() {
        m_poseEstimator.update(m_gyroAngleSupplier.get(), m_swerveModulePositionsSupplier.get());
        m_field.setRobotPose(m_poseEstimator.getEstimatedPosition());
        SmartDashboard.putData("Odometry/Field", m_field);
    }
}
