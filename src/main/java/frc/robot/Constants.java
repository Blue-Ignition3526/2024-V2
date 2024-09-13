package frc.robot;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class Intake {
    public static final double kInSpeed = -0.5d;
  }

  public static final class Indexer {
    public static final int kIndexerEncoderPort = 0;

    public static final TrapezoidProfile.Constraints kIndexerConstraints = new TrapezoidProfile.Constraints(10, 12);
    public static final ProfiledPIDController kIndexerPIDController = new ProfiledPIDController(1, 0, 0, kIndexerConstraints);
  }
}
