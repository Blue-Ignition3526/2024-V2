package frc.robot.commands.Climber.Single;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Climber;
import lib.BlueShift.commands.RunForCommand;

public class ClimberHome extends SequentialCommandGroup {
  public ClimberHome(Climber climber) {
    addCommands(
      new RunForCommand(new ClimberUp(climber), 0.1),
      new ClimberFullyDown(climber),
      new RunForCommand(new ClimberDown(climber), 0.15)
    );
  }
}
