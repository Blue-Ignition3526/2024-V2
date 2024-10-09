package main.java.frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class HighHeightCommand extends CommandBase {
    private final ElevatorSubsystem elevator;

    public HighHeightCommand(ElevatorSubsystem elevator) {
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.goToHighHeight();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(elevator.getCurrentHeight() - 120.0) < 1.0; // Adjust tolerance as needed
    }

    @Override
    public void end(boolean interrupted) {
        elevator.stopElevator();
    }
}
