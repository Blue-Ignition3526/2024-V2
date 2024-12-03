package frc.robot.subsystems.Gyro;
import edu.wpi.first.math.geometry.Rotation2d;

public interface GyroIO {
    double getPitch();
    double getYaw();
    double getRoll();

    double getPitchVelocity();
    double getYawVelocity();
    double getRollVelocity();

    double getAccelerationX();
    double getAccelerationY();
    double getAccelerationZ();

    Rotation2d getHeading();

    void reset();
    
    void setYaw(double yawDeg);

    default void periodic() {};
}
