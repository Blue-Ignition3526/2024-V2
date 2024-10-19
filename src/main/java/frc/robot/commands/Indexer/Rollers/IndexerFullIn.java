package frc.robot.commands.Indexer.Rollers;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IndexerRollers;

public class IndexerFullIn extends Command {
  private IndexerRollers rollers; //crea variable rollers del subsistema IndexRollers

  public void Indexer(IndexerRollers rollers) {
    this.rollers = rollers; // permite que la clase tenga acceso a este subsistema
    addRequirements(rollers); //deja que otros comandos que utilizen rollers puedan usarse pero le da prioridad a este cuando se usa
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() { //siempre
    rollers.setRollersSpeed(Constants.Indexer.Rollers.krollersInReceivingSpeed); // pone rollers a full recibir
  }

  @Override
  public void end(boolean interrupted) { // al final del comando
    rollers.stop(); // para los rollers
  }

  @Override
  public boolean isFinished() { //cuando el comando acabe
    return false;
  }
}
