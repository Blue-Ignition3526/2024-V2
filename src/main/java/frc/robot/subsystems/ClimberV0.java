package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import lib.BlueShift.control.motor.LazyCANSparkMax;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ProfiledPIDController;

public class ClimberV0 extends SubsystemBase {
    String name;

    private final LazyCANSparkMax rightClimberMotor;
    private final LazyCANSparkMax leftClimberMotor;
    private final RelativeEncoder climberEncoder;
    private final ProfiledPIDController climberMotorPID;
    private double climberSetPoint;

    public ClimberV0(){
        this.leftClimberMotor = new LazyCANSparkMax(Constants.Climber.leftClimberMotorID, MotorType.kBrushless);
        this.rightClimberMotor = new LazyCANSparkMax(Constants.Climber.rightClimberMotorID, MotorType.kBrushless);
        this.rightClimberMotor.follow(this.leftClimberMotor, true);
        this.climberEncoder = leftClimberMotor.getEncoder();
        this.climberMotorPID = Constants.Climber.kclimberPIDController;
        this.climberEncoder.setPositionConversionFactor(Constants.Climber.kclimberEncoder_RotationToInches); 
        this.climberEncoder.setVelocityConversionFactor(Constants.Climber.kclimberEncoder_RPMToInchesPerSecond);
        climberMotorPID.reset(getPosition());
        // coast just for debugging
        leftClimberMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightClimberMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        SmartDashboard.putData("CLIMBER_PID", Constants.Climber.kclimberPIDController);

    }
    
    //TODO: A base de posiciones


    public void setPosition(double position) {
        climberSetPoint = position;
    }

    public double getPosition() {
        return climberEncoder.getPosition();
    }

    public void stop() {
        leftClimberMotor.set(0);
    }

    public double getCurrent() {
        return leftClimberMotor.getOutputCurrent();
    }

    public Command setClimberPositionCommand(double position) {
        return run(() -> setPosition(position));
    } 

    @Override
    public void periodic(){
        double voltage = climberMotorPID.calculate(getPosition(),climberSetPoint);
        leftClimberMotor.setVoltage(voltage);
        SmartDashboard.putNumber("encoder", getPosition());
        SmartDashboard.putNumber("voltage", voltage);
        SmartDashboard.putNumber("percentageR", rightClimberMotor.getAppliedOutput());
    }
}