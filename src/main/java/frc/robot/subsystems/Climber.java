package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import lib.BlueShift.control.motor.LazyCANSparkMax;
import static edu.wpi.first.units.Units.Inches;

public class Climber extends SubsystemBase {
  enum ClimberPosition {
    HIGH(Inches.of(15)),
    LOW(Inches.of(0));

    private final Measure<Distance> position;

    ClimberPosition(Measure<Distance> position) {
      this.position = position;
    }

    public Measure<Distance> getPosition() {
      return position;
    }
  }

  String name;

  private final LazyCANSparkMax climberMotor;
  private final RelativeEncoder climberEncoder;
  private ClimberPosition climberSetPoint;
  
  public Climber(int motorId) {
    // Motor
    this.climberMotor = new LazyCANSparkMax(motorId, MotorType.kBrushless);
    this.climberMotor.setIdleMode(IdleMode.kBrake);

    // Encoder
    this.climberEncoder = climberMotor.getEncoder();
    this.climberEncoder.setPositionConversionFactor(Constants.Climber.kclimberEncoder_RotationToInches); 
    this.climberEncoder.setVelocityConversionFactor(Constants.Climber.kclimberEncoder_RPMToInchesPerSecond);

    // PID
    Constants.Climber.kclimberPIDController.reset(climberEncoder.getPosition());
  }

  /**
   * Get the climber's current position
   * @return
   */
  public Measure<Distance> getPosition() {
    return Inches.of(climberEncoder.getPosition());
  }

  /**
   * Get the climber's setpoint
   * @param setpoint
   */
  public Measure<Distance> getSetpoint() {
    return climberSetPoint.getPosition();
  }

  @Override
  public void periodic() {
    
  }
}
