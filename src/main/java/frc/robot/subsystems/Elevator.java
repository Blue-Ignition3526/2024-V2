package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import lib.BlueShift.control.motor.LazyCANSparkFlex;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Elevator extends SubsystemBase {
    public static enum ElevatorPosition {
        HIGH("High", Inches.of(8.0)),
        MEDIUM("Medium", Inches.of(5.0)),
        LOW("Low", Inches.of(0.0));
  
        private final String name;
        private final Measure<Distance> position;
  
        ElevatorPosition(String name, Measure<Distance> position){
          this.name = name;
          this.position = position;
        }
  
        public String getName(){
          return name;
        }
  
        public Measure<Distance> getPosition(){
          return position;
        }
    }

    // * Motors
    private final LazyCANSparkFlex m_leftElevatorMotor;
    private final LazyCANSparkFlex m_rightElevatorMotor;

    // * Encoder
    private final RelativeEncoder m_encoder;

    // * Setpoint
    private ElevatorPosition m_setpoint = ElevatorPosition.LOW;

    // * Logging
    private DoubleLogEntry m_currentPositionLog;
    private DoubleLogEntry m_setpointPositionLog;
    private DoubleLogEntry m_setVoltageLog;
    private DoubleLogEntry m_setpointErrorLog;
    private BooleanLogEntry m_atSetpointLog;

    public Elevator () {
        // ! LEFT MOTOR: MASTER
        // ! RIGHT MOTOR: FOLLOWER
        // TODO: Current limits
        // TODO: Set to brake when PID works
        // Motors
        // Left motor
        this.m_leftElevatorMotor = new LazyCANSparkFlex(Constants.Elevator.kLeftElevatorMotorId, MotorType.kBrushless);
        this.m_leftElevatorMotor.setIdleMode(IdleMode.kCoast);

        // Right motor
        this.m_rightElevatorMotor = new LazyCANSparkFlex(Constants.Elevator.kRightElevatorMotorId, MotorType.kBrushless);
        this.m_rightElevatorMotor.setIdleMode(IdleMode.kCoast);

        // Configure right motor as an inverted follower
        this.m_rightElevatorMotor.follow(this.m_leftElevatorMotor, true);

        // Encoder
        this.m_encoder = m_leftElevatorMotor.getEncoder();

        // Reset PID controller
        Constants.Elevator.kElevatorPIDController.reset(getPosition().in(Inches));

        // Logging
        DataLog dataLog = DataLogManager.getLog();
        m_currentPositionLog = new DoubleLogEntry(dataLog, "Elevator/CurrentPosition");
        m_setpointPositionLog = new DoubleLogEntry(dataLog, "Elevator/SetpointPosition");
        m_setVoltageLog = new DoubleLogEntry(dataLog, "Elevator/SetVoltage");
        m_setpointErrorLog = new DoubleLogEntry(dataLog, "Elevator/SetpointError");
        m_atSetpointLog = new BooleanLogEntry(dataLog, "Elevator/AtSetpoint");
    }

    /**
     * Get the position of the elevator (in inches)
     * @return
     */
    public Measure<Distance> getPosition() {
        // encoder (rotations) * 2pi (radians) * radiansToInches
        return Inches.of(this.m_encoder.getPosition() * 2 * Math.PI * Constants.Elevator.kRadiansToInches);
    }

    /**
     * Set the setpoint position for the elevator
     * @param setpoint
     */
    public void setSetpointPosition(ElevatorPosition setpoint) {
        this.m_setpoint = setpoint;
    }

    /**
     * Get the error between the setpoint and the current position
     * @return
     */
    public Measure<Distance> getSetpointError() {
        return this.m_setpoint.getPosition().minus(getPosition());
    }

    /**
     * Get the setpoint position for the elevator
     * @return
     */
    public ElevatorPosition getSetpoint() {
        return this.m_setpoint;
    }

    /**
     * Check if the elevator is at the setpoint
     * @return
     */
    public boolean atSetpoint() {
        return Math.abs(getSetpointError().in(Inches)) < Constants.Elevator.kElevatorTolerance.in(Inches);
    }

    /**
     * Stop elevator
     */
    public void stop() {
        this.m_leftElevatorMotor.set(0);
    }

    /**
     * Set the setpoint position for the elevator
     * @param position
     * @return
     */
    public Command setPositionCommand(ElevatorPosition position) {
        return run(() -> setSetpointPosition(position)).until(() -> atSetpoint());
    }

    public Command resetPIDCommand() {
        return runOnce(() -> Constants.Elevator.kElevatorPIDController.reset(getPosition().in(Inches)));
    }

    @Override
    public void periodic() {
        // From here on, calculations done in inches
        double currentPosition = getPosition().in(Inches);
        double setpointPosition = m_setpoint.getPosition().in(Inches);

        // Calculate set voltage
        double setVoltage = 0;
        if (!atSetpoint()) {
            double pid = Constants.Elevator.kElevatorPIDController.calculate(currentPosition, setpointPosition);
            setVoltage = pid;
        } else {
            setVoltage = 0;
        }

        // Update motor voltage
        // (left motor should follow)
        m_leftElevatorMotor.setVoltage(setVoltage);

        // Update SmartDashboard
        SmartDashboard.putNumber("Elevator/CurrentPosition", currentPosition);
        SmartDashboard.putNumber("Elevator/SetpointPosition", setpointPosition);
        SmartDashboard.putNumber("Elevator/SetVoltage", setVoltage);
        SmartDashboard.putNumber("Elevator/SetpointError", getSetpointError().in(Inches));
        SmartDashboard.putBoolean("Elevator/AtSetpoint", atSetpoint());

        // Update logging
        m_currentPositionLog.append(currentPosition);
        m_setpointPositionLog.append(setpointPosition);
        m_setVoltageLog.append(setVoltage);
        m_setpointErrorLog.append(getSetpointError().in(Inches));
        m_atSetpointLog.append(atSetpoint());
    }
}
