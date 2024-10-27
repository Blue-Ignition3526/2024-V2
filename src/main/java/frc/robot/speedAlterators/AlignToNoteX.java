package frc.robot.speedAlterators;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
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

    @Override
    public ChassisSpeeds alterSpeed(ChassisSpeeds speeds, boolean robotRelative) {
        ChassisSpeeds modifiedSpeeds = robotRelative ? speeds : ChassisSpeeds.fromFieldRelativeSpeeds(speeds, headingSupplier.get());
        double xOffsetDeg = LimelightHelpers.getTX(limelightName);
        modifiedSpeeds.vxMetersPerSecond = Constants.SwerveDrive.kTranslationController.calculate(xOffsetDeg, 0);
        return modifiedSpeeds;
    }
}
