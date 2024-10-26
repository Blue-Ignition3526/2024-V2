package frc.robot.speedAlterators;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.LimelightHelpers;
import lib.BlueShift.control.SpeedAlterator;

public class AlignToNoteX extends SpeedAlterator {
    Supplier<Rotation2d> headingSupplier;
    String limelightName;
    int lastPipelineIndex;
    int noteTrackerPipelineIndex;

    public AlignToNoteX(Supplier<Rotation2d> headingSupplier, String limelightName, int noteTrackerPipelineIndex) {
        this.headingSupplier = headingSupplier;
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
        ChassisSpeeds robotRelativeChassisSpeeds = robotRelative ? speeds : ChassisSpeeds.fromFieldRelativeSpeeds(speeds, headingSupplier.get());
        
        return speeds;
    }
}
