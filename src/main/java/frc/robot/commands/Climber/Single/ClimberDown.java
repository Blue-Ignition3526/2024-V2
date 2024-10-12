package frc.robot.commands.Climber.Single;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ClimberDown extends Command {
  Climber m_climber;

  public ClimberDown(Climber climber) {
    this.m_climber = climber;
    addRequirements(climber);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_climber.setClimberDown();
  }

  @Override
  public void end(boolean interrupted) {
    m_climber.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}