package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import lib.BlueShift.control.motor.LazyCANSparkFlex;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import lib.BlueShift.control.motor.LazySparkPID;


public class Shooter extends SubsystemBase {
    // * Upper Roller
    LazyCANSparkFlex upperRoller;
    LazySparkPID upperRollerPID;
    RelativeEncoder upperRollerEncoder;

    // * Lower Roller
    LazyCANSparkFlex lowerRoller;
    LazySparkPID lowerRollerPID;
    RelativeEncoder lowerRollerEncoder;

    // * Control
    Measure<Velocity<Angle>> velocitySetpoint = RPM.of(0.);

    // * Logging
    DoubleLogEntry upperRollerVelocityLog;
    DoubleLogEntry lowerRollerVelocityLog;
    DoubleLogEntry setpointVelocityLog;
    BooleanLogEntry atSetpointLog;

    public Shooter() {
        // * Upper Roller
        // Motor
        this.upperRoller = new LazyCANSparkFlex(Constants.Shooter.kUpperRollerID, MotorType.kBrushless);
        this.upperRoller.setIdleMode(IdleMode.kCoast);
        this.upperRoller.setSmartCurrentLimit(Constants.Shooter.UpperSmartCurrentLimit);
        this.upperRoller.setOpenLoopRampRate(Constants.Shooter.UpperClosedLoopRampRate);
        this.upperRoller.setClosedLoopRampRate(Constants.Shooter.UpperClosedLoopRampRate);
        this.upperRoller.setInverted(true);

        // PID
        this.upperRollerPID = new LazySparkPID(this.upperRoller);

        // Encoder
        this.upperRollerEncoder = this.upperRoller.getEncoder();

        // * Lower Roller
        // Motor
        this.lowerRoller = new LazyCANSparkFlex(Constants.Shooter.kLowerRollerID, MotorType.kBrushless);
        this.lowerRoller.setIdleMode(IdleMode.kCoast);
        this.lowerRoller.setSmartCurrentLimit(Constants.Shooter.LowerSmartCurrentLimit);
        this.lowerRoller.setOpenLoopRampRate(Constants.Shooter.lowerClosedLoopRampRate);
        this.lowerRoller.setClosedLoopRampRate(Constants.Shooter.lowerClosedLoopRampRate);        
        this.lowerRoller.setInverted(true);


        // PID
        this.lowerRollerPID  = new LazySparkPID(lowerRoller);

        // Encoder
        this.lowerRollerEncoder = this.lowerRoller.getEncoder();

        // * Logging
        DataLog dataLog = DataLogManager.getLog();
        this.upperRollerVelocityLog = new DoubleLogEntry(dataLog, "Shooter/UpperRollerVelocity");
        this.lowerRollerVelocityLog = new DoubleLogEntry(dataLog, "Shooter/LowerRollerVelocity");
        this.setpointVelocityLog = new DoubleLogEntry(dataLog, "Shooter/SetpointVelocity");
        this.atSetpointLog = new BooleanLogEntry(dataLog, "Shooter/AtSetpoint");
    }

    /**
     * Set the velocity of the upper roller
     * @return
     */
    public double getUpperRollerVelocity() {
        return upperRollerEncoder.getVelocity();
    }

    /**
     * Set the velocity of the lower roller
     * @return
     */
    public double getLowerRollerVelocity() {
        return lowerRollerEncoder.getVelocity();
    }

    /**
     * Get the error between the current velocity and the setpoint for the upper roller
     * @return
     */
    public double getUpperRollerSetpointError() {
        return getUpperRollerVelocity() - velocitySetpoint.in(RPM);
    }

    /**
     * Get the error between the current velocity and the setpoint for the lower roller
     * @return
     */
    public double getLowerRollerSetpointError() {
        return getLowerRollerVelocity() - velocitySetpoint.in(RPM);
    }

    /**
     * Check if the upper roller is at the setpoint
     * @return
     */
    public boolean upperRollerAtSetpoint() {
        return Math.abs(getUpperRollerSetpointError()) <= Constants.Shooter.kVelocityToleranceRPM;
    }

    /**
     * Check if the lower roller is at the setpoint
     * @return
     */
    public boolean lowerRollerAtSetpoint() {
        return Math.abs(getLowerRollerSetpointError()) <= Constants.Shooter.kVelocityToleranceRPM;
    }

    /**
     * Check if both rollers are at the setpoint
     * @return
     */
    public boolean atSetpoint() {
        return upperRollerAtSetpoint() && lowerRollerAtSetpoint();
    }

    /**
     * Set the velocity of both rollers
     * @param rpm
     */
    public void setSetpointRPM(Measure<Velocity<Angle>> rpm) {
        this.velocitySetpoint = rpm;
        upperRollerPID.setReference(velocitySetpoint.in(RPM), ControlType.kSmartVelocity);
        lowerRollerPID.setReference(velocitySetpoint.in(RPM), ControlType.kSmartVelocity);
    }

    public void setVelocity(double percentage) {
        this.upperRoller.set(percentage);
        this.lowerRoller.set(percentage);
    }

    public void setVelocity(double percentageTop, double percentageBottom) {
        this.upperRoller.set(percentageTop);
        this.lowerRoller.set(percentageBottom);
    }

    /**
     * Stop both rollers
     */
    public void stop() {
        setVelocity(0);
    }

    /**
     * Set the velocity of both rollers
     * THIS COMMAND DOES NOT WAIT
     * @param rpm the velocity to set
     * @return
     */
    public Command setRpmCommand(Measure<Velocity<Angle>> rpm) {
        return run(() -> setSetpointRPM(rpm)).until(() -> atSetpoint());
    }

    public Command setVelocityCommand(double percentage) {
        return runOnce(() -> setVelocity(percentage));
    }

    public Command setVelocityCommand(double percentageTop, double percentageBottom) {
        return runOnce(() -> setVelocity(percentageTop, percentageBottom));
    }

    /**
     * Set the velocity of both rollers
     * @param rpm the velocity to set
     * @return
     */
    public Command setRpmCommand(double rpm) {
        return run(() -> setSetpointRPM(RPM.of(rpm))).until(() -> atSetpoint());
    }

    /**
     * Set the velocity of both rollers
     * @param rpm the velocity to set
     * @return
     */
    public Command stopCommand() {
        return run(() -> stop());
    }

    /**
     * Set the velocity of both rollers to the idle speed
     * @return
     */
    public Command setIdleCommand() {
        return runOnce(() -> setVelocity(0.2));
    }

    public Command setShootCommand() {
        return runOnce(() -> setVelocity(0.7,0.9));
    }

    public Command setPassCommand() {
        return runOnce(() -> setVelocity(0.9));
    }

    @Override
    public void periodic() {
        // * Logging
        SmartDashboard.putNumber("Shooter/UpperRollerVelocity", getUpperRollerVelocity());
        SmartDashboard.putNumber("Shooter/LowerRollerVelocity", getLowerRollerVelocity());
        SmartDashboard.putNumber("Shooter/Setpoint", velocitySetpoint.in(RPM));
        SmartDashboard.putBoolean("Shooter/AtSetpoint", atSetpoint());

        upperRollerVelocityLog.append(getUpperRollerVelocity());
        lowerRollerVelocityLog.append(getLowerRollerVelocity());
        setpointVelocityLog.append(velocitySetpoint.in(RPM));
        atSetpointLog.append(atSetpoint());
    }
}
