package frc.robot.subsystems.Gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gyro extends SubsystemBase implements GyroIO {
  private GyroIO io;
  private double vx, vy, vz = 0d;
  private double lastTime = Timer.getFPGATimestamp();

  /**
   * Creates a new Gyro. 
   **/
  public Gyro(GyroIO io) {
    this.io = io;
  }

  /**
   * Gets the pitch of the gyro.
   * @return
   */
  public double getPitch() {
    return io.getPitch();
  }

  /**
   * Gets the yaw of the gyro.
   * @return
   */
  public double getYaw() {
    return io.getYaw();
  }

  /**
   * Gets the roll of the gyro.
   * @return
   */
  public double getRoll() {
    return io.getRoll();
  }

  /**
   * Gets the pitch velocity of the gyro.
   * @return
   */ 
  public double getPitchVelocity() {
    return io.getPitchVelocity();
  }

  /**
   * Gets the yaw velocity of the gyro.
   * @return
   */
  public double getYawVelocity() {
    return io.getYawVelocity();
  }

  /**
   * Gets the roll velocity of the gyro.
   * @return
   */
  public double getRollVelocity() {
    return io.getRollVelocity();
  }

  /**
   * Gets the acceleration of the gyro in the X direction.
   * @return
   */
  public double getAccelerationX() {
    return io.getAccelerationX();
  }

  /**
   * Gets the acceleration of the gyro in the Y direction.
   * @return
   */
  public double getAccelerationY() {
    return io.getAccelerationY();
  }

  /**
   * Gets the acceleration of the gyro in the Z direction.
   * @return
   */
  public double getAccelerationZ() {
    return io.getAccelerationZ();
  }

  public Rotation2d getHeading() {
    return io.getHeading();
  }

  /**
   * Resets the gyro.
   */
  public void reset() {
    io.reset();
  }

  /*
   * Sets gyro yaw
   */
  public void setYaw(double yawDeg) {
    io.setYaw(yawDeg);
  }

  /**
   * Gets the velocity of the gyro in the X direction. (ROBOT RELATIVE)
   * @return
   */
  public double getVelocityX() {
    return vx;
  }
  
  /**
   * Gets the velocity of the gyro in the Y direction. (ROBOT RELATIVE)
   * @return
   */
  public double getVelocityY() {
    return vy;
  }

  /**
   * Gets the velocity of the gyro in the Z direction. (ROBOT RELATIVE)
   * @return
   */
  public double getVelocityZ() {
    return vz;
  }

  /**
   * Gets the velocity of the gyro in the X and Y directions. (ROBOT RELATIVE)
   * @return
   */
  public Translation2d getPlanarVelocity() {
    return new Translation2d(vx, vy);
  }

  public void periodic() {
    // Calculate velocity
    double currentTime = Timer.getFPGATimestamp();
    double dt = currentTime - lastTime;
    lastTime = currentTime;

    vx += io.getAccelerationX() * dt;
    vy += io.getAccelerationY() * dt;
    vz += io.getAccelerationZ() * dt;
  }
}
