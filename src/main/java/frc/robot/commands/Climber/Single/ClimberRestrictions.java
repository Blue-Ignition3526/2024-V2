package frc.robot.commands.Climber.Single;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerPivot;
import frc.robot.Constants;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ClimberRestrictions extends Command {
    private final CANSparkMax climberMotor;
    private final ElevatorSubsystem elevator;
    private final IndexerPivot pivot;

    public ClimberRestrictions(int motorID, ElevatorSubsystem elevator, IndexerPivot pivot, CANSparkMax climberMotor) {
        this.climberMotor = new CANSparkMax(motorID, MotorType.kBrushless);
        this.elevator = elevator;
        this.pivot = pivot;
    }

    public void setClimberUp() {
        // Solo permitir movimiento si las condiciones son adecuadas
        if (elevator.isAtHighHeight() && pivot.isPivotAtShooterAngle()) {
            climberMotor.set(Constants.Climber.kClimberUpSpeed);
        } else {
            stop();
        }
    }

    public void setClimberDown() {
        climberMotor.set(Constants.Climber.kClimberDownSpeed);
    }

    public void stop() {
        climberMotor.set(0);
    }
}