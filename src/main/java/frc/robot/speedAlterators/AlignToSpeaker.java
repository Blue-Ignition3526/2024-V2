package frc.robot.speedAlterators;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import lib.BlueShift.control.SpeedAlterator;

public class AlignToSpeaker extends SpeedAlterator {
    Supplier<Pose2d> poseSupplier;
    Alliance alliance;

    public AlignToSpeaker(Supplier<Pose2d> poseSupplier) {
        this.poseSupplier = poseSupplier;
    }

    @Override
    public void onEnable() {
        alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    }

    public ChassisSpeeds alterSpeed(ChassisSpeeds speeds, boolean robotRelative) {
        Pose3d speakerPose = alliance == Alliance.Blue ? Constants.Field.kBlueSpeakerPoseMeters : Constants.Field.kRedSpeakerPoseMeters;
        Pose2d robotPose = poseSupplier.get();
        double angle = Math.toDegrees(Math.atan2(speakerPose.getY() - robotPose.getY(), speakerPose.getX() - robotPose.getX())) + 180;
        double omega = Constants.SwerveDrive.kHeadingController.calculate(robotPose.getRotation().getRadians(), angle);
        return new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, omega);
    }
}
