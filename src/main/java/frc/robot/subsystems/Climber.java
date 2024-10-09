package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;

public class Climber extends SubsystemBase {
    CANSparkMax climberMotor;
    String name;

    public Climber(int motorID, String name) {
        this.name = name;
        this.climberMotor = new CANSparkMax(motorID, MotorType.kBrushless);
    }

    public void set(double speed) {
        climberMotor.set(speed);
    }

    public void setClimberUp() {
        climberMotor.set(Constants.Climber.kClimberUpSpeed);
    }

    public void setClimberDown() {
        climberMotor.set(Constants.Climber.kClimberDownSpeed);
    }

    public void stop() {
        climberMotor.set(0);
    }

    public double getCurrent() {
        return climberMotor.getOutputCurrent();
    }

    // TODO: Get the height of the climbers
    // TODO: Add enums for climber positions
    public Measure<Distance> getPosition() {
        return null;
    }
}