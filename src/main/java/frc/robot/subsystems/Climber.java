package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import lib.BlueShift.control.motor.LazyCANSparkMax;
import lib.BlueShift.control.motor.LazySparkPID;

import org.w3c.dom.views.DocumentView;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ProfiledPIDController;

//TODO: Hacer PID 

public class Climber extends SubsystemBase {
    String name;

    private final LazyCANSparkMax leftClimberMotor;
    private final LazyCANSparkMax rightClimberMotor;
    private final RelativeEncoder climberEncoder;
    private final ProfiledPIDController climberMotorPID;  
    private double climberSetPoint;  

    public Climber(){
        this.rightClimberMotor = new LazyCANSparkMax(Constants.Climber.leftClimberMotorID, MotorType.kBrushless);
        this.leftClimberMotor = new LazyCANSparkMax(Constants.Climber.rightClimberMotorID, MotorType.kBrushless);
        this.leftClimberMotor.follow(this.rightClimberMotor);
        this.climberEncoder = rightClimberMotor.getEncoder();
        this.climberMotorPID = Constants.Climber.kclimberPIDController;
        this.climberEncoder.setPositionConversionFactor(Constants.Climber.kclimberEncoder_RotationToInches); 
        this.climberEncoder.setVelocityConversionFactor(Constants.Climber.kclimberEncoder_RPMToInchesPerSecond);

        // coast just for debugging
        rightClimberMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        leftClimberMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);

    }
    
    //TODO: A base de posiciones


    public void setPosition(double position) {
        climberSetPoint = position;
    }

    public double getPosition() {
        return climberEncoder.getPosition();
    }

    public void stop() {
        rightClimberMotor.set(0);
    }

    public double getCurrent() {
        return rightClimberMotor.getOutputCurrent();
    }

    /*public Command setClimberPosition(double position) {
        return new run(() -> setPosition(position), this);
    } */

    @Override
    public void periodic(){
        rightClimberMotor.setVoltage(climberMotorPID.calculate(getPosition(),climberSetPoint));
    }

    // TODO: Get the height of the climbers
    // TODO: Add enums for climber positions
}