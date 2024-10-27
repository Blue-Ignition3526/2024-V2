package frc.robot.commands.SwerveDrive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive;

public class XFormation extends Command {
  SwerveDrive swerveDrive;

  public XFormation(SwerveDrive swerveDrive) {
    this.swerveDrive = swerveDrive;
    addRequirements(swerveDrive);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    this.swerveDrive.xFormation();
  }

  @Override
  public void end(boolean interrupted) {
    this.swerveDrive.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
