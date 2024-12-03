package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber.ClimberPosition;

public class ClimberCommands {
    public static Command setClimbersLowCommand(Climber leftClimber, Climber rightClimber) {
        return new ParallelCommandGroup(
            leftClimber.setPositionCommand(ClimberPosition.LOW),
            rightClimber.setPositionCommand(ClimberPosition.LOW)
        );
    }

    public static Command setClimbersHighCommand(Climber leftClimber, Climber rightClimber) {
        return new ParallelCommandGroup(
            leftClimber.setPositionCommand(ClimberPosition.HIGH),
            rightClimber.setPositionCommand(ClimberPosition.HIGH)
        );
    }
}
