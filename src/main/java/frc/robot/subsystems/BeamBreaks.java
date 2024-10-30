package frc.robot.subsystems;

import java.util.HashMap;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BeamBreaks extends SubsystemBase {
  /**
   * Enum for the beam breaks on the robot.
   */
  public static enum BeamBreak {
    INDEXER_STAGE2("Indexer2", Constants.BreamBreaks.kIndexerStage2BeamBreakPort, true);

    private final String name;
    private final int channel;
    private final boolean inverted;

    BeamBreak(String name, int channel, boolean inverted) {
      this.name = name;
      this.channel = channel;
      this.inverted = inverted;
    }

    public String getName() {
      return this.name;
    }

    public int getChannel() {
      return this.channel;
    }

    public boolean isInverted() {
      return this.inverted;
    }
  }

  private final HashMap<String, DigitalInput> m_beamBreaks = new HashMap<>(BeamBreak.values().length);
  private boolean enabled = true;

  public BeamBreaks() {
    for (BeamBreak beamBreak : BeamBreak.values()) m_beamBreaks.put(beamBreak.getName(), new DigitalInput(beamBreak.getChannel()));
  }

  /**
   * Get the value of a beam break. If the beam break is inverted, the value will be negated.
   * When disabled this will return false regardless of wether the beam break is inverted.
   * @param beamBreak
   * @return
   */
  public boolean get(BeamBreak beamBreak) {
    if (!enabled) return false;
    boolean value = m_beamBreaks.get(beamBreak.getName()).get();
    return beamBreak.isInverted() ? !value : value;
  }

  /**
   * Disable the beam breaks. When disabled, all beam breaks will return false.
   */
  public void disable() {
    enabled = false;
    SmartDashboard.putBoolean("BeamBreaks/Enabled", enabled);
  }

  /**
   * Enable the beam breaks.
   */
  public void enable() {
    enabled = true;
    SmartDashboard.putBoolean("BeamBreaks/Enabled", enabled);
  }

  /**
   * Check if the beam breaks are enabled.
   * @return
   */
  public boolean isEnabled() {
    return enabled;
  }

  @Override
  public void periodic() {
    for (BeamBreak beamBreak : BeamBreak.values()) SmartDashboard.putBoolean("BeamBreaks/" + beamBreak.getName(), get(beamBreak));
  }
}
