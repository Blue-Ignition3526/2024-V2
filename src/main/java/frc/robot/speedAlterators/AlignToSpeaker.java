package frc.robot.speedAlterators;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import lib.BlueShift.control.SpeedAlterator;

public class AlignToSpeaker extends SpeedAlterator {
    Supplier<Pose2d> poseSupplier;
    Alliance alliance;

    public AlignToSpeaker(Supplier<Pose2d> poseSupplier) {
        this.poseSupplier = poseSupplier;
        SmartDashboard.putData("AlignToSpeakerPID", Constants.SwerveDrive.kHeadingController);
    }

    @Override
    public void onEnable() {
        alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        Constants.SwerveDrive.kHeadingController.reset(poseSupplier.get().getRotation().getRadians());
    }

    public ChassisSpeeds alterSpeed(ChassisSpeeds speeds, boolean robotRelative) {
        Pose3d speakerPose = alliance == Alliance.Blue ? Constants.Field.kBlueSpeakerPoseMeters : Constants.Field.kRedSpeakerPoseMeters;
        Pose2d robotPose = poseSupplier.get();
        double angle = Math.atan2(speakerPose.getY() - robotPose.getY(), speakerPose.getX() - robotPose.getX());
        double omega = Constants.SwerveDrive.kHeadingController.calculate(robotPose.getRotation().getRadians(), angle);
        SmartDashboard.putNumber("SpeedAlterator/Omega", omega);
        return new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, omega);
    }
}
