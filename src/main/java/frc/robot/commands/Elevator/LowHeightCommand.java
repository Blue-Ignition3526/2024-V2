package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class LowHeightCommand extends CommandBase {
    private final ElevatorSubsystem elevator;

    public LowHeightCommand(ElevatorSubsystem elevator) {
        this.elevator = elevator;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.goToLowHeight();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(elevator.getCurrentHeight() - 10.0) < 1.0; // Adjust tolerance as needed
    }

    @Override
    public void end(boolean interrupted) {
        elevator.stopElevator();
    }
}
