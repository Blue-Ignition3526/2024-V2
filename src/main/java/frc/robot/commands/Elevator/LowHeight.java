package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class LowHeight extends Command{
    private Elevator elevator; 
    private double desiredHeight = Constants.Elevator.klowerBound;

    public LowHeight (Elevator elevator) {
    this.elevator = elevator; 
    addRequirements(elevator); 
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() { //siempre
    elevator.setPosition(desiredHeight);
  }

  @Override
  public void end(boolean interrupted) { // al final del comando
    elevator.stopElevator(); // para los rollers
  }

  @Override
  public boolean isFinished() { //cuando el comando acabe
    return false;
  }
}
