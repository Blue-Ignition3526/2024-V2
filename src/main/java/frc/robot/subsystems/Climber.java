package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import lib.BlueShift.control.motor.LazyCANSparkMax;
import static edu.wpi.first.units.Units.Inches;

public class Climber extends SubsystemBase {
  public static enum ClimberPosition {
    HIGH(Inches.of(10)),
    LOW(Inches.of(0));

    private final Measure<Distance> position;

    ClimberPosition(Measure<Distance> position) {
      this.position = position;
    }

    public Measure<Distance> getPosition() {
      return position;
    }
  }

  String name;

  private final LazyCANSparkMax climberMotor;
  private final RelativeEncoder climberEncoder;
  private ClimberPosition climberSetPoint;

  DoubleLogEntry m_currentPositionLog;
  DoubleLogEntry m_setpointPositionLog;
  DoubleLogEntry m_setVoltageLog;
  BooleanLogEntry m_atSetpointLog;
  
  public Climber(String name, int motorId) {
    // Display name
    this.name = name;

    // Motor
    this.climberMotor = new LazyCANSparkMax(motorId, MotorType.kBrushless);
    this.climberMotor.setIdleMode(IdleMode.kBrake);

    // Encoder
    this.climberEncoder = climberMotor.getEncoder();
    this.climberEncoder.setPositionConversionFactor(Constants.Climber.kclimberEncoder_RotationToInches); 
    this.climberEncoder.setVelocityConversionFactor(Constants.Climber.kclimberEncoder_RPMToInchesPerSecond);

    // PID
    Constants.Climber.kclimberPIDController.reset(climberEncoder.getPosition());

    // Logging
    DataLog dataLog = DataLogManager.getLog();
    m_currentPositionLog = new DoubleLogEntry(dataLog, name + "/CurrentPosition");
    m_setpointPositionLog = new DoubleLogEntry(dataLog, name + "/SetpointPosition");
    m_setVoltageLog = new DoubleLogEntry(dataLog, name + "/SetVoltage");
    m_atSetpointLog = new BooleanLogEntry(dataLog, name + "/AtSetpoint");
  }

  public void setSetpoint(ClimberPosition setpoint) {
    this.climberSetPoint = setpoint;
  }

  /**
   * Get the climber's current position
   * @return
   */
  public Measure<Distance> getPosition() {
    return Inches.of(climberEncoder.getPosition());
  }

  /**
   * Get the climber's setpoint
   * @param setpoint
   */
  public Measure<Distance> getSetpoint() {
    return climberSetPoint.getPosition();
  }

  /**
   * Get the setpoint error
   * @return
   */
  public Measure<Distance> getSetpointError() {
    return climberSetPoint.getPosition().minus(climberSetPoint.getPosition());
  }

  /**
   * Check the climber's setpoint
   * @return
   */
  public boolean atSetpoint() {
    return getSetpointError().in(Inches) <= Constants.Climber.kClimberTolerance.in(Inches);
  }

  public Command resetPIDCommand() {
    return runOnce(() -> Constants.Climber.kclimberPIDController.reset(getPosition().in(Inches)));
  }

  public Command setPositionCommand(ClimberPosition position) {
    return run(() -> setSetpoint(position)).until(() -> atSetpoint());
  }

  @Override
  public void periodic() {
    double voltage = Constants.Climber.kclimberPIDController.calculate(getPosition().in(Inches), climberSetPoint.getPosition().in(Inches));
    climberMotor.setVoltage(voltage);

    // SmartDashboard
    SmartDashboard.putNumber(name + "/CurrentPosition", getPosition().in(Inches));
    SmartDashboard.putNumber(name + "/SetpointPosition", getSetpoint().in(Inches));
    SmartDashboard.putNumber(name + "/SetVoltage", voltage);
    SmartDashboard.putBoolean(name + "/AtSetpoint", atSetpoint());


    // Logging
    m_currentPositionLog.append(getPosition().in(Inches));
    m_setpointPositionLog.append(getSetpoint().in(Inches));
    m_setVoltageLog.append(voltage);
    m_atSetpointLog.append(atSetpoint());
  }
}
