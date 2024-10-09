package frc.robot.commands.Indexer.Pivot;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.IndexerPivot;

public class AlignIndexerPivotToSpeaker extends Command {
  IndexerPivot indexerPivot;
  Elevator elevator;
  Supplier<Pose2d> poseSupplier;

  /**
   * ELEVATOR IS NOT REQUIRED, ONLY USED FOR DATA
   * @param indexerPivot
   * @param elevator
   */
  public AlignIndexerPivotToSpeaker(IndexerPivot indexerPivot, Elevator elevator, Supplier<Pose2d> poseSupplier) {
    this.indexerPivot = indexerPivot;
    this.elevator = elevator;
    this.poseSupplier = poseSupplier;
    addRequirements(indexerPivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // TODO: Check if it's possible to use this in initialize
    if (LimelightHelpers.getCurrentPipelineIndex(Constants.Vision.Limelight3G.kName) != Constants.Vision.Limelight3G.kSpeakerPipeline) {
      LimelightHelpers.setPipelineIndex(Constants.Vision.Limelight3G.kName, Constants.Vision.Limelight3G.kSpeakerPipeline);
    }

    // TODO: Calculate indexer height
    double h_i = 0;
    double h_f = Constants.Field.kBlueSpeakerPoseMeters.getZ();
    double g = 9.81;
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
