// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private final CANSparkMax motor;

  public Intake(int id) {
    this.motor = new CANSparkMax(id, MotorType.kBrushless);
    this.motor.setIdleMode(IdleMode.kCoast);
  }

  public void im() {
    this.motor.set(Constants.Intake.kInSpeed);

  }

  public void stop() {
    this.motor.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
