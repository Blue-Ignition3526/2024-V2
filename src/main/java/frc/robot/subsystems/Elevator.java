// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;


public class Elevator extends SubsystemBase {

    private final CANSparkFlex leftElevatorMotor;
    private final CANSparkFlex rightElevatorMotor;

    private final RelativeEncoder encoder;

    private final PIDController elevatorPID;

    // TODO: Cambiarlo al verdadero valor
    private final double radiansTometers = 1.0; // How many rotations are how many inches in height 
    
    // Constructor
    public Elevator () {
        this.leftElevatorMotor = new CANSparkFlex(Constants.Elevator.kleftElevatorMotorId, MotorType.kBrushless);
        this.rightElevatorMotor = new CANSparkFlex(Constants.Elevator.krightElevatorMotorId, MotorType.kBrushless);

        rightElevatorMotor.follow(leftElevatorMotor);

        this.encoder = leftElevatorMotor.getEncoder();

        this.elevatorPID = new PIDController(Constants.Elevator.kElevatorP, Constants.Elevator.kElevatorI, Constants.Elevator.kElevatorD);

    }

    public void setSpeed(double speed) {
        leftElevatorMotor.set(speed);
    }   

    public void moveUp() {
        setSpeed(Constants.Elevator.kspeedUp);
    }

    public void moveDown() {
        setSpeed(Constants.Elevator.kspeedDown);
    }

    public void keepInPlace() {
        setSpeed(Constants.Elevator.kspeedInPlace);
    }

    public void setPosition(double position){
        setSpeed(elevatorPID.calculate(position, getPosition()));
    }

    public double getPosition () {
        return (2*Math.PI/encoder.getPosition()) * radiansTometers;
    }


    public void stopElevator() {
        setSpeed(0.0);
    }

    @Override
    public void periodic() {
        // Optional: Add code here to report height to dashboard or log data
    }
}