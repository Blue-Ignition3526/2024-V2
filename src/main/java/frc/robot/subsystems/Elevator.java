package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Elevator.ElevatorPosition;
import edu.wpi.first.math.controller.ProfiledPIDController;


public class Elevator extends SubsystemBase {
    // * Motors
    private final CANSparkFlex leftElevatorMotor;
    private final CANSparkFlex rightElevatorMotor;

    // * Encoder
    private final RelativeEncoder encoder;

    // Constructor
    public Elevator () {
        this.leftElevatorMotor = new CANSparkFlex(Constants.Elevator.kleftElevatorMotorId, MotorType.kBrushless);
        this.rightElevatorMotor = new CANSparkFlex(Constants.Elevator.krightElevatorMotorId, MotorType.kBrushless);

        rightElevatorMotor.setInverted(true);

        leftElevatorMotor.follow(rightElevatorMotor);

        this.encoder = rightElevatorMotor.getEncoder();
    
    }

    public void setSpeed(double speed) {
        rightElevatorMotor.set(speed);
    }   

    public void setPosition(double position){
        setSpeed(Constants.Elevator.kElevatorPIDController.calculate(getPosition(), position));
    }

    public double getPosition () {
        return (2*Math.PI/encoder.getPosition()) * Constants.Elevator.kradiansToInches;
    }


    public void stopElevator() {
        setSpeed(0.0);
    }

    public Command setPositionCommand(ElevatorPosition position) {
        return run(()-> setPosition(position.getPosition())).until(()->Math.abs(position.getPosition()-getPosition())<0.5);
    }

    @Override
    public void periodic() {
        // Optional: Add code here to report height to dashboard or log data
    }
}