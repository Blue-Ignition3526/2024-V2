package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class MidHeightCommand extends CommandBase {
    private final ElevatorSubsystem elevator;

    public MidHeightCommand(ElevatorSubsystem elevator) {
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.goToMidHeight();
    }

    @Override
    public boolean isFinished() {
        // You can add a condition to check if the elevator is near the target
        return Math.abs(elevator.getCurrentHeight() - 60.0) < 1.0; // Adjust tolerance as needed
    }

    @Override
    public void end(boolean interrupted) {
        elevator.stopElevator();
    }
}

