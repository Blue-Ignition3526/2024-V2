package frc.robot.commands.SwerveDrive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive;

public class ResetTurningEncoders extends Command {
  SwerveDrive swerveDrive;

  public ResetTurningEncoders(SwerveDrive swerveDrive) {
    this.swerveDrive = swerveDrive;
    addRequirements(swerveDrive);
  }

  @Override
  public void initialize() {
    this.swerveDrive.stop();
  }

  @Override
  public void execute() {
    this.swerveDrive.resetTurningEncoders();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return true;
  }
}
