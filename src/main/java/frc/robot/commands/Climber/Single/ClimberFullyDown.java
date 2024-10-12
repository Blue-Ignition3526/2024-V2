package frc.robot.commands.Climber.Single;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class ClimberFullyDown extends Command {
  private final Climber climber;

  public ClimberFullyDown(Climber climber) {
    this.climber = climber;
    addRequirements(climber);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    this.climber.setClimberDown();
  }

  @Override
  public void end(boolean interrupted) {
    this.climber.stop();
  }

  @Override
  public boolean isFinished() {
    return this.climber.getCurrent() >= Constants.Climber.kMaxCurrent;
  }
}
