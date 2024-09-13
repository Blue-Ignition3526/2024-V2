package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import lib.BlueShift.control.motor.LazyCANSparkMax;

public class Indexer extends SubsystemBase {
  private final DutyCycleEncoder indexerEncoder;
  private final LazyCANSparkMax indexerMotor;

  public Indexer() {
    this.indexerEncoder = new DutyCycleEncoder(Constants.Indexer.kIndexerEncoderPort);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
