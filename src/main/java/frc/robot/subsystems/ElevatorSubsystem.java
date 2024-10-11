// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkFlexExternalEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
    private final CANSparkFlex motor;
    private final DutyCycleEncoder encoder;

    private final double kP = 0.1;
    private final double kI = 0.0;
    private final double kD = 0.0;

    private final double lowHeight = 10.0;   // Inches
    private final double midHeight = 60.0;   // Inches
    private final double highHeight = 120.0; // Inches

    private final double maxHeight = 130.0;  // Max elevator height in inches
    private final double minHeight = 0.0;    // Min elevator height

    private final double motorRotationsToInches = 1.0; // Adjust based on gear ratios, etc.

    // Constructor
    public ElevatorSubsystem() {
        motor = new CANSparkFlex(1,MotorType.kBrushless); // Motor PWM channel, adjust as needed
        encoder = new DutyCycleEncoder(0); // PWM port for the encoder

        // Set encoder distance per rotation (adjust based on your system)
        encoder.setDistancePerRotation(motorRotationsToInches);
    }

    public void goToLowHeight() {
        setElevatorHeight(lowHeight);
    }

    public void goToMidHeight() {
        setElevatorHeight(midHeight);
    }

    public void goToHighHeight() {
        setElevatorHeight(highHeight);
    }

    
    private void setElevatorHeight(double heightInches) {
        double setpoint = inchesToMotorRotations(heightInches);
        if (setpoint < inchesToMotorRotations(minHeight)) {
            setpoint = inchesToMotorRotations(minHeight);
        } else if (setpoint > inchesToMotorRotations(maxHeight)) {
            setpoint = inchesToMotorRotations(maxHeight);
        }

        // Apply proportional control for simplicity (replace with PID if needed)
        double error = setpoint - getCurrentHeight();
        double output = kP * error;
        
        // Ensure the output is between -1 and 1 (motor limits)
        if (output > 1) {
            output = 1;
        } else if (output < -1) {
            output = -1;
        }

        motor.set(output);
    }

    private double inchesToMotorRotations(double inches) {
        return inches / motorRotationsToInches;
    }

    public double getCurrentHeight() {
        return encoder.getDistance();
    }
    
    public boolean isAtHighHeight() {
        return getCurrentHeight() >= highHeight;
    }

    public void stopElevator() {
        motor.set(0.0);
    }

    @Override
    public void periodic() {
        // Optional: Add code here to report height to dashboard or log data
    }
}