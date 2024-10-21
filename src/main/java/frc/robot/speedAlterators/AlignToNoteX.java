package frc.robot.speedAlterators;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.LimelightHelpers;
import lib.BlueShift.control.SpeedAlterator;

public class AlignToNoteX extends SpeedAlterator {
    String limelightName;
    int lastPipelineIndex;
    int noteTrackerPipelineIndex;

    public AlignToNoteX(String limelightName, int noteTrackerPipelineIndex) {
        this.limelightName = limelightName;
        this.noteTrackerPipelineIndex = noteTrackerPipelineIndex;
    }

    @Override
    public void onEnable() {
        this.lastPipelineIndex = (int)LimelightHelpers.getCurrentPipelineIndex(limelightName);
        LimelightHelpers.setPipelineIndex(limelightName, noteTrackerPipelineIndex);
    }

    @Override
    public void onDisable() {
        LimelightHelpers.setPipelineIndex(limelightName, lastPipelineIndex);
    }

    // TODO: CHECK LOGIC
    @Override
    public ChassisSpeeds alterSpeed(ChassisSpeeds speeds, boolean robotRelative) {
        return speeds;
    }
}
