package frc.robot.subsystems;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import lib.BlueShift.control.motor.LazyCANSparkMax;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotation;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class IndexerPivot extends SubsystemBase {
  public static enum IndexerPivotPosition {
    REST(Degrees.of(0)),
    RECEIVING(Degrees.of(5));

    Measure<Angle> angle;

    IndexerPivotPosition(Measure<Angle> angle) {
      this.angle = angle;
    }

    public Measure<Angle> getAngle() {
      return angle;
    }
  }

  // Encoder
  private final DutyCycleEncoder indexerEncoder;

  // Motor
  private final LazyCANSparkMax m_pivotMotor;

  // Setpoint
  private Measure<Angle> m_setpoint;

  // Logging
  DoubleLogEntry m_currentAngleLog;
  DoubleLogEntry m_setpointAngleLog;
  DoubleLogEntry m_setVoltageLog;
  DoubleLogEntry m_setpointErrorLog;

  public IndexerPivot() {
    // Encoder
    this.indexerEncoder = new DutyCycleEncoder(Constants.Indexer.Pivot.kIndexerPivotEncoderPort);
    this.indexerEncoder.setPositionOffset(Constants.Indexer.Pivot.kIndexerPivotEncoderOffset.in(Rotation));
    
    // Motor
    this.m_pivotMotor = new LazyCANSparkMax(Constants.Indexer.Pivot.kIndexerPivotMotorId, MotorType.kBrushless);
    this.m_pivotMotor.setIdleMode(IdleMode.kBrake);
    this.m_pivotMotor.setSmartCurrentLimit(Constants.Indexer.Pivot.kIndexerPivotMotorMaxCurrent);

    // Reset setpoint
    this.m_setpoint = getAngle();

    // Reset PID controller
    Constants.Indexer.Pivot.kIndexerPivotPIDController.reset(getAngle().in(Degrees));

    // Logging
    DataLog dataLog = DataLogManager.getLog();
    m_currentAngleLog = new DoubleLogEntry(dataLog, "Indexer/Pivot/CurrentAngle");
    m_setpointAngleLog = new DoubleLogEntry(dataLog, "Indexer/Pivot/SetpointAngle");
    m_setVoltageLog = new DoubleLogEntry(dataLog, "Indexer/Pivot/SetVoltage");
    m_setpointErrorLog = new DoubleLogEntry(dataLog, "Indexer/Pivot/SetpointError");

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
   * @return the angle of the indexer pivot
   */
  public Measure<Angle> getAngle() {
    // TODO: Add offset
    return Rotation.of(indexerEncoder.getAbsolutePosition()).times(Constants.Indexer.Pivot.kIndexerPivotEncoderRatio);
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
   * @return the setpoint error
   */
  public Measure<Angle> getSetpointError() {
    return m_setpoint.minus(getAngle());
  }

  /**
   * Check if the indexer pivot is at the setpoint
   * @return true if the indexer pivot is at the setpoint
   */
  public boolean atSetpoint() {
    return Math.abs(getSetpointError().in(Degrees)) < Constants.Indexer.Pivot.kIndexerPivotTolerance.in(Degrees);
  }

  /**
   * Stop the indexer pivot
   */
  public void stop() {
    m_pivotMotor.set(0);
  }

  // TODO: Commands
  public Command resetPIDCommand() {
    return runOnce(() -> Constants.Indexer.Pivot.kIndexerPivotPIDController.reset(getAngle().in(Degrees)));
  }

  public Command setSetpointCommand(Measure<Angle> setpoint) {
    return run(() -> setSetpointAngle(setpoint)).until(() -> atSetpoint());
  }

  public Command setSetpointCommand(IndexerPivotPosition setpoint) {
    return setSetpointCommand(setpoint.getAngle());
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
      // TODO: If the setpoint is not reached, apply Feedforward
      double pid = Constants.Indexer.Pivot.kIndexerPivotPIDController.calculate(currentAngle, setpointAngle);
      // double ff = Constants.Indexer.Pivot.kIndexerPivotFeedforward.calculate(pid);
      setVoltage = pid;
    } else {
      // TODO: Check for weird behaviors
      setVoltage = 0;
    }

    // Update motor voltage
    m_pivotMotor.setVoltage(setVoltage);
    
    // Update SmartDashboard
    SmartDashboard.putNumber("Indexer/Pivot/CurrentAngle", currentAngle);
    SmartDashboard.putNumber("Indexer/Pivot/SetpointAngle", setpointAngle);
    SmartDashboard.putNumber("Indexer/Pivot/SetVoltage", setVoltage);
    SmartDashboard.putNumber("Indexer/Pivot/SetpointError", getSetpointError().in(Degrees));
    SmartDashboard.putBoolean("Indexer/Pivot/AtSetpoint", atSetpoint());

    // Update logging
    m_currentAngleLog.append(currentAngle);
    m_setpointAngleLog.append(setpointAngle);
    m_setVoltageLog.append(setVoltage);
    m_setpointErrorLog.append(getSetpointError().in(Degrees));
  }
}
