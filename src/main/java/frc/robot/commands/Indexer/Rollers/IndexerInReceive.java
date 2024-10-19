package frc.robot.commands.Indexer.Rollers;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IndexerRollers;

public class IndexerInReceive extends Command {
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
    rollers.setRollersSpeed(Constants.Indexer.Rollers.krollersInReceivingSpeed); // pone los rollers a la velocidad que estableciste previamente para cuando los rollers agarran
  }

  @Override
  public void end(boolean interrupted) { // al final del comando
    if (rollers.hasPieceIn()); // me falta agregar que la limelight parpadee // si los rollers tienen pieza la limelight parpadea, Necesita la libreria que crearon para funcionar
    rollers.stop(); // para los rollers
  }

  @Override
  public boolean isFinished() { //cuando el comando acabe
    return rollers.hasPieceIn();// regresa que si los rollers tienen pieza o no
  }
}

