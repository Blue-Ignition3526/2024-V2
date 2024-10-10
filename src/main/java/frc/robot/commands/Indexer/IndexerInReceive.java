package frc.robot.commands.Indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IndexerRollers;
import lib.BlueShift.utils.LimelightLED;

public class IndexerInReceive extends Command {
  private IndexerRollers rollersMotorIn; //crea variable rollersIn del subsistema IntakeRollers

  public void IndexIn(IndexerRollers rollersMotorIn) {
    this.rollersMotorIn = rollersMotorIn; // permite que la clase tenga acceso a este subsistema
    addRequirements(rollersMotorIn); //deja que otros comandos que utilizen rollers puedan usarse pero le da prioridad a este cuando se usa
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() { //siempre
    rollersMotorIn.setRollersInReceivingSpeed(); // pone los rollers a la velocidad que estableciste previamente para cuando los rollers agarran
  }

  @Override
  public void end(boolean interrupted) { // al final del comando
    if (rollersMotorIn.hasPieceIn()) LimelightLED.blinkLeds(Constants.Vision.Limelight3.kName); // si los rollers tienen pieza la limelight parpadea, Necesita la libreria que crearon para funcionar
    rollersMotorIn.stop(); // para los rollers
  }

  @Override
  public boolean isFinished() { //cuando el comando acabe
    return rollersMotorIn.hasPieceIn();// regresa que si los rollers tienen pieza o no
  }
}
