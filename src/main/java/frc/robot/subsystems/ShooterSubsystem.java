package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkBase.IdleMode;

import lib.BlueShift.control.motor.LazyCANSparkFlex;
import lib.BlueShift.control.motor.LazyCANSparkMax;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import lib.BlueShift.control.motor.LazySparkPID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ShooterSubsystem extends SubsystemBase {
    // * Upper Roller
    LazyCANSparkFlex upperRoller;
    LazySparkPID upperRollerPID;

    // * Lower Roller
    LazyCANSparkFlex lowerRoller;
    LazySparkPID lowerRollerPID;

    Measure<Velocity<Angle>> velocitySetpoint = RPM.of(0.);

    public ShooterSubsystem() {
        this.upperRoller = new LazyCANSparkFlex(Constants.ShooterSubsystem.kUpperRollerID, MotorType.kBrushless);
        this.upperRoller.setIdleMode(IdleMode.kCoast);
       
        this.upperRoller.setSmartCurrentLimit(Constants.ShooterSubsystem.UpperSmartCurrentLimit);
        this.upperRoller.setClosedLoopRampRate(Constants.ShooterSubsystem.UpperClosedLoopRampRate);
        this.upperRoller.setInverted(true);

        this.upperRollerPID = new LazySparkPID(this.upperRoller);

        this.lowerRoller = new LazyCANSparkFlex(Constants.ShooterSubsystem.kLowerRollerID, MotorType.kBrushless);
        this.lowerRoller.setIdleMode(IdleMode.kCoast);
        
        this.lowerRoller.setSmartCurrentLimit(Constants.ShooterSubsystem.LowerSmartCurrentLimit);
        this.lowerRoller.setClosedLoopRampRate(Constants.ShooterSubsystem.lowerClosedLoopRampRate);

        this.lowerRollerPID = new LazySparkPID(this.lowerRoller);
    }
    //TODO: quitar sets
    //TODO: un spark siga a otro
    //TODO: hacer comando en linea para RPM
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
