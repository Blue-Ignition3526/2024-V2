package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import lib.BlueShift.control.motor.LazyCANSparkFlex;
import lib.BlueShift.control.motor.LazyCANSparkMax;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Elevator extends SubsystemBase {
    // * Motors
    private final LazyCANSparkMax m_leftElevatorMotor;
    private final LazyCANSparkMax m_rightElevatorMotor;

    // * Encoder
    private final RelativeEncoder m_encoder;

    // * Setpoint
    private double m_setpoint = 0.0;

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
        this.m_leftElevatorMotor = new LazyCANSparkMax(Constants.Elevator.kLeftElevatorMotorId, MotorType.kBrushless);
        this.m_leftElevatorMotor.setIdleMode(IdleMode.kBrake);
        this.m_leftElevatorMotor.setInverted(false);

        // Right motor
        this.m_rightElevatorMotor = new LazyCANSparkMax(Constants.Elevator.kRightElevatorMotorId, MotorType.kBrushless);
        this.m_rightElevatorMotor.setIdleMode(IdleMode.kBrake);

        // Configure right motor as an inverted follower
        this.m_rightElevatorMotor.follow(this.m_leftElevatorMotor, true);

        // Encoder
        this.m_encoder = m_rightElevatorMotor.getEncoder();

        // Reset PID controller
        Constants.Elevator.kElevatorPIDController.reset(getPosition());

        // LOG PID
        SmartDashboard.putData("Elevator/PIDController", Constants.Elevator.kElevatorPIDController);

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
    public double getPosition() {
        return this.m_encoder.getPosition();
    }

    /**
     * Set the setpoint position for the elevator
     * @param setpoint
     */
    public void setSetpointPosition(double setpoint) {
        this.m_setpoint = setpoint;
    }

    /**
     * Get the error between the setpoint and the current position
     * @return
     */
    public double getSetpointError() {
        return Math.abs(this.m_setpoint)-(getPosition());
    }

    /**
     * Get the setpoint position for the elevator
     * @return
     */
    public double getSetpoint() {
        return this.m_setpoint;
    }

    /**
     * Check if the elevator is at the setpoint
     * @return
     */
    public boolean atSetpoint() {
        return Math.abs(getSetpointError()) < Constants.Elevator.kElevatorTolerance;
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
    public Command setPositionCommand(double position) {
        return runOnce(() -> m_setpoint = position);
    }

    public Command resetPIDCommand() {
        return runOnce(() -> Constants.Elevator.kElevatorPIDController.reset(getPosition()));
    }

    @Override
    public void periodic() {
        // From here on, calculations done in inches
        double currentPosition = getPosition();
        double setpointPosition = m_setpoint;

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
        SmartDashboard.putNumber("Elevator/CurrentPosition", getPosition());
        SmartDashboard.putNumber("Elevator/SetpointPosition", setpointPosition);
        SmartDashboard.putNumber("Elevator/SetVoltage", setVoltage);
        SmartDashboard.putNumber("Elevator/SetpointError", getSetpointError());
        SmartDashboard.putBoolean("Elevator/AtSetpoint", atSetpoint());
                SmartDashboard.putNumber("Elevator/LeftAmperage", m_leftElevatorMotor.getOutputCurrent());
                SmartDashboard.putNumber("Elevator/RightAmperage", m_rightElevatorMotor.getOutputCurrent());



        // Update logging
        m_currentPositionLog.append(currentPosition);
        m_setpointPositionLog.append(setpointPosition);
        m_setVoltageLog.append(setVoltage);
        m_setpointErrorLog.append(getSetpointError());
        m_atSetpointLog.append(atSetpoint());
    }
}
