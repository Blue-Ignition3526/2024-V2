package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import lib.BlueShift.control.motor.LazyCANSparkMax;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotation;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class IndexerPivot extends SubsystemBase {
  // Encoder
  private final DutyCycleEncoder indexerEncoder;

  // Motor
  private final LazyCANSparkMax m_pivotMotor;

  // Control
  private Measure<Angle> m_setpoint;

  public IndexerPivot() {
    // Encoder
    this.indexerEncoder = new DutyCycleEncoder(Constants.Indexer.Pivot.kIndexerPivotEncoderPort);
    this.indexerEncoder.setPositionOffset(Constants.Indexer.Pivot.kIndexerPivotEncoderOffset.in(Rotation));
    
    // Reset setpoint
    this.m_setpoint = getAngle();
    
    // Motor
    this.m_pivotMotor = new LazyCANSparkMax(Constants.Indexer.Pivot.kIndexerPivotMotorId, MotorType.kBrushless);
    this.m_pivotMotor.setIdleMode(IdleMode.kBrake);
    this.m_pivotMotor.setSmartCurrentLimit(Constants.Indexer.Pivot.kIndexerPivotMotorMaxCurrent);

    // Control
    SmartDashboard.putData("Indexer/Pivot/PIDController", Constants.Indexer.Pivot.kIndexerPivotPIDController);
  }

  // TODO: Implement emergency home and use motor encoder if absolute encoder fails

  /**
   * Check if the encoder is connected
   * @return true if the encoder is connected
   */
  public boolean isEncoderConnected() {
    return indexerEncoder.isConnected();
  }

  /**
   * Get the angle reported by the Absolute through-bore encoder (in rotation)
   * RANGE: -180 to 180 degrees
   * @return the angle of the indexer pivot
   */
  public Measure<Angle> getAngle() {
    // TODO: Add offset
    return Rotation.of(indexerEncoder.getAbsolutePosition()).minus(Degrees.of(180));
  }

  /**
   * Set the setpoint angle for the indexer pivot
   * @param setpoint the setpoint to set
   */
  public void setSetpointAngle(Measure<Angle> setpoint) {
    m_setpoint = setpoint;
  }

  /**
   * Get the setpoint angle for the indexer pivot
   * @return the setpoint angle
   */
  public double getSetpointError() {
    return getAngle().in(Degrees) - m_setpoint.in(Degrees);
  }

  /**
   * Check if the indexer pivot is at the setpoint
   * @return true if the indexer pivot is at the setpoint
   */
  public boolean atSetpoint() {
    return Math.abs(getSetpointError()) < Constants.Indexer.Pivot.kIndexerPivotTolerance.in(Degrees);
  }

  @Override
  public void periodic() {
    // TODO: BE VERY CAREFUL AND TEST MANUALLY FIRST
    // From here on, calculations done in degrees
    
    // Get current angle and setpoint angle
    double currentAngle = getAngle().in(Degrees);
    double setpointAngle = m_setpoint.in(Degrees);
    
    // Calculate set voltage
    double setVoltage = 0;
    if (!atSetpoint()) {
      SmartDashboard.putBoolean("Indexer/Pivot/AtSetpoint", false);
      // TODO: If the setpoint is not reached, apply Feedforward
      double pid = Constants.Indexer.Pivot.kIndexerPivotPIDController.calculate(currentAngle, setpointAngle);
      // double ff = Constants.Indexer.Pivot.kIndexerPivotFeedforward.calculate(pid);
      setVoltage = pid;
    } else {
      // TODO: Check for weird behaviors
      SmartDashboard.putBoolean("Indexer/Pivot/AtSetpoint", true);
      setVoltage = 0;
    }

    // Update motor voltage
    m_pivotMotor.setVoltage(setVoltage);
    
    // Update SmartDashboard
    SmartDashboard.putNumber("Indexer/Pivot/CurrentAngle", currentAngle);
    SmartDashboard.putNumber("Indexer/Pivot/SetpointAngle", setpointAngle);
    SmartDashboard.putNumber("Indexer/Pivot/SetVoltage", setVoltage);
    SmartDashboard.putNumber("Indexer/Pivot/SetpointError", getSetpointError());
  }
}
