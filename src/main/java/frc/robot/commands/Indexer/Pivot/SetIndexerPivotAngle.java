package frc.robot.commands.Indexer.Pivot;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerPivot;

public class SetIndexerPivotAngle extends Command {
  IndexerPivot indexerPivot;
  Measure<Angle> angle;

  public SetIndexerPivotAngle(IndexerPivot indexerPivot, Measure<Angle> angle) {
    this.indexerPivot = indexerPivot;
    this.angle = angle;
    addRequirements(indexerPivot);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    indexerPivot.setSetpointAngle(angle);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return indexerPivot.atSetpoint();
  }
}
