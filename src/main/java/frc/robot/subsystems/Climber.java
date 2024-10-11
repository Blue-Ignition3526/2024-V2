package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;

public class Climber extends SubsystemBase {
    String name;

    private final CANSparkMax climberMotor;
    private final ElevatorSubsystem elevator;
    private final IndexerPivot pivot;

    public Climber(int motorID, String name,ElevatorSubsystem elevator, IndexerPivot pivot) {
        this.name = name;
        this.climberMotor = new CANSparkMax(motorID, MotorType.kBrushless);
        this.elevator = elevator;
        this.pivot = pivot;
    }

    public void set(double speed) {
        climberMotor.set(speed);
    }

    public void setClimberUp() {
    if (elevator.isAtHighHeight() && pivot.isPivotAtShooterAngle()) {
        climberMotor.set(Constants.Climber.kClimberUpSpeed);
    }
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
}