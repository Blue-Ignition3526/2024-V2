package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import lib.BlueShift.control.motor.LazyCANSparkFlex;

public class Intake extends SubsystemBase {
  // Motor
  private final LazyCANSparkFlex m_motor;

  public Intake() {
    // Motor
    this.m_motor = new LazyCANSparkFlex(Constants.Intake.kMotorId, MotorType.kBrushless);
    this.m_motor.setIdleMode(IdleMode.kCoast);
    this.m_motor.setSmartCurrentLimit(Constants.Intake.kMotorMaxCurrent);
  }

  /**
   * Set the intake to intake
   * @return void
   */
  public void setIn() {
    this.m_motor.set(Constants.Intake.kInSpeed);
  }

  /**
   * Set the intake to outtake
   * @return void
   */
  public void setOut() {
    this.m_motor.set(Constants.Intake.kOutSpeed);
  }

  /**
   * Set the intake to avoid (sets to a slow out speed to avoid jamming)
   * @return void
   */
  public void setAvoid() {
    this.m_motor.set(Constants.Intake.kAvoidSpeed);
  }

  /**
   * Stop the intake
   * @return void
   */
  public void setStop() {
    this.m_motor.stopMotor();
  }

  @Override
  public void periodic() {}
}
