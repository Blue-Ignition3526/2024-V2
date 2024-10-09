package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkBase.IdleMode;
import lib.BlueShift.control.motor.LazyCANSparkMax;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ShooterSubsystem extends SubsystemBase {
    CANSparkFlex upperRoller;
    CANSparkFlex lowerRoller;

    Measure<Velocity<Angle>> velocitySetpoint = RPM.of(0.);

public ShooterSubsystem() {
    this.upperRoller = new CANSparkFlex(Constants.ShooterSubsystem.kUpperRollerID, MotorType.kBrushless);
    this.lowerRoller = new CANSparkFlex(Constants.ShooterSubsystem.kLowerRollerID, MotorType.kBrushless);
}
public void setUpperSpeed(double speed) {
    upperRoller.set(speed);
}
public void setLowerSpeed(double speed) {
    lowerRoller.set(speed);
}
public void shoot(double upperSpeed, double lowerSpeed) {
    setUpperSpeed(upperSpeed);
    setLowerSpeed(lowerSpeed);
}

public void stop() {
    shoot(0, 0);
}

@Override
public void periodic() {
   // double projectileSpeed = calculateProjectileSpeed(distance); // Calcular velocidad del proyectil
    //double force = power / projectileSpeed; // Calcular fuerza

   // SmartDashboard.putNumber("Force of Fire", force); // Mostrar en SmartDashboard
}
}
