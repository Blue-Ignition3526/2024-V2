package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkBase.ControlType;
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
    RelativeEncoder upperRollerEncoder;

    // * Lower Roller
    LazyCANSparkFlex lowerRoller;
    LazySparkPID lowerRollerPID;
    RelativeEncoder lowerRollerEncoder;

    Measure<Velocity<Angle>> velocitySetpoint = RPM.of(0.);

    public ShooterSubsystem() {
        this.upperRoller = new LazyCANSparkFlex(Constants.ShooterSubsystem.kUpperRollerID, MotorType.kBrushless);
        this.upperRoller.setIdleMode(IdleMode.kCoast);
        this.upperRoller.setSmartCurrentLimit(Constants.ShooterSubsystem.UpperSmartCurrentLimit);
        this.upperRoller.setClosedLoopRampRate(Constants.ShooterSubsystem.UpperClosedLoopRampRate);
        this.upperRoller.setInverted(true);
        this.upperRollerPID = new LazySparkPID(this.upperRoller);
        this.upperRollerEncoder = this.upperRoller.getEncoder();

        this.lowerRoller = new LazyCANSparkFlex(Constants.ShooterSubsystem.kLowerRollerID, MotorType.kBrushless);
        this.lowerRoller.setIdleMode(IdleMode.kCoast);
        this.lowerRoller.setSmartCurrentLimit(Constants.ShooterSubsystem.LowerSmartCurrentLimit);
        this.lowerRoller.setClosedLoopRampRate(Constants.ShooterSubsystem.lowerClosedLoopRampRate);
        this.lowerRollerPID  = new LazySparkPID(lowerRoller);
        this.lowerRollerEncoder = this.lowerRoller.getEncoder();
    }
    //TODO: quitar sets (listo creo)
    //TODO: un spark siga a otro (listo creo)
    //TODO: hacer comando en linea para RPM
    public void setUpperSpeed(double speed) {
    }

    public void setLowerSpeed(double speed) {

    }

    public void setTargetUpperSpeed(double finalSpeed){

    }

    public void setTargetLowerSpeed(double finalSpeed){

    }

    public void setShooterSpeed(double upperSpeed, double lowerSpeed) {
        upperRoller.set(0);
        lowerRoller.set(0);
        //setUpperSpeed(upperSpeed);
        //setLowerSpeed(lowerSpeed);
        
    }


    public void stop() {
        setShooterSpeed(0, 0);
    }

    public Command setRpm(Measure<Velocity<Angle>> rpm) {
        return run(() -> {
            upperRollerPID.setReference(rpm.in(RPM), ControlType.kVelocity);
            lowerRollerPID.setReference(rpm.in(RPM), ControlType.kVelocity);
        })
        .until(() -> {
            return 
                Math.abs(upperRollerEncoder.getVelocity() - rpm.in(RPM)) <= Constants.ShooterSubsystem.kVelocityToleranceRPM &&
                Math.abs(lowerRollerEncoder.getVelocity() - rpm.in(RPM)) <= Constants.ShooterSubsystem.kVelocityToleranceRPM;
        }).;
    }

    @Override
    public void periodic() {
        // double projectileSpeed = calculateProjectileSpeed(distance); // Calcular velocidad del proyectil
        //double force = power / projectileSpeed; // Calcular fuerza
        // SmartDashboard.putNumber("Force of Fire", force); // Mostrar en SmartDashboard
    }
}
