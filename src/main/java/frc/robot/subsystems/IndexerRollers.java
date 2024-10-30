package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.BeamBreaks.BeamBreak;

public class IndexerRollers extends SubsystemBase {
    // * Motor
    private final CANSparkMax rollersMotor;

    // * Beam Breaks instance
    BeamBreaks beamBreaks;

    public IndexerRollers(BeamBreaks beamBreaks) {
        // Motor
        this.rollersMotor = new CANSparkMax(Constants.Indexer.Rollers.kRollersMotorInID, MotorType.kBrushless);
        this.rollersMotor.setSmartCurrentLimit(Constants.Indexer.Rollers.krollersCurrentLimiterInAmps);

        // Beam Breaks
        this.beamBreaks = beamBreaks;
    }

    /**
     * Set the rollers to coast idle mode (used when no note detected)
     * @return
     */
    public Command setRollersCoastCommand() {
        SmartDashboard.putString("Indexer/IdleMode", "Coast");
        return runOnce(() -> this.rollersMotor.setIdleMode(CANSparkMax.IdleMode.kCoast));
    }

    /**
     * Set the rollers to brake idle mode (used when note detected)
     * @return
     */
    public Command setRollersBrakeCommand() {
        SmartDashboard.putString("Indexer/IdleMode", "Brake");
        return runOnce(() -> this.rollersMotor.setIdleMode(CANSparkMax.IdleMode.kBrake));
    }

    /**
     * Set the rollers to a specific speed
     * @param speed
     * @return
     */
    public Command setRollersSpeedCommand(double speed) {
        SmartDashboard.putNumber("Indexer/Speed", speed);
        return run(() -> this.rollersMotor.set(speed));
    }

    /**
     * Stop the rollers
     * @return
     */
    public Command stopRollersCommand() {
        return setRollersSpeedCommand(0);
    }

    /**
     * Set the rollers to intake
     * @return
     */
    public Command setRollersInCommand() {
        return setRollersCoastCommand().andThen(setRollersSpeedCommand(Constants.Indexer.Rollers.krollersInReceivingSpeed));
    }

    /**
     * Intakes until second beam break is broken
     * @return
     */
    public Command receiveCommand() {
        return 
            setRollersInCommand()
            .until(() -> beamBreaks.get(BeamBreak.INDEXER_STAGE2))
            .andThen(
                setRollersBrakeCommand(),
                stopRollersCommand()
            );
    }

    /**
     * Set the rollers to outtake
     * @return
     */
    public Command setRollersOutCommand() {
        return setRollersCoastCommand().andThen(setRollersSpeedCommand(Constants.Indexer.Rollers.krollersInExpulsingSpeed));
    }

    /**
     * Set the rollers to pass
     * @return
     */
    public Command setRollersPassCommand() {
        return setRollersCoastCommand().andThen(setRollersSpeedCommand(Constants.Indexer.Rollers.krollersInPassSpeed));
    }

    @Override
    public void periodic() {
        // SmartDashboard.putBoolean("Indexer/BeamBreak1", beamBreaks.get(BeamBreak.INDEXER_STAGE1));
        SmartDashboard.putBoolean("Indexer/BeamBreak2", beamBreaks.get(BeamBreak.INDEXER_STAGE2));
    }
}
