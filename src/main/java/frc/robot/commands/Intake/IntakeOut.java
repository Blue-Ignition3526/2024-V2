package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeOut extends Command {
  Intake m_intake;

  public IntakeOut(Intake intake) {
    this.m_intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    this.m_intake.setOut();
  }

  @Override
  public void end(boolean interrupted) {
    this.m_intake.setStop(); 
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
