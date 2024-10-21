package frc.robot.subsystems;

import java.util.HashMap;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// TODO: Checar esta estructura, la neta no me gusto 💀
public class BeamBreaks extends SubsystemBase {
  private final HashMap<String, DigitalInput> m_beamBreaks = new HashMap<>();

  public enum BeamBreak {
    INTAKE("Intake", Constants.BreamBreaks.kIntakeBeamBreakPort),
    INDEXER_STAGE1("Indexer1", Constants.BreamBreaks.kIndexerStage1BeamBreakPort),
    INDEXER_STAGE2("Indexer2", Constants.BreamBreaks.kIndexerStage2BeamBreakPort);

    private final String name;
    private final int channel;

    BeamBreak(String name, int channel) {
      this.name = name;
      this.channel = channel;
    }

    public String getName() {
      return this.name;
    }

    public int getChannel() {
      return this.channel;
    }
  }

  public BeamBreaks() {
    for (BeamBreak beamBreak : BeamBreak.values()) {
      m_beamBreaks.put(beamBreak.getName(), new DigitalInput(beamBreak.getChannel()));
    }
  }

  public boolean get(BeamBreak beamBreak) {
    return m_beamBreaks.get(beamBreak.getName()).get();
  }

  @Override
  public void periodic() {
    m_beamBreaks.forEach((name, beamBreak) -> {
      SmartDashboard.putData("BeamBreaks/" + name, beamBreak);
    });
  }
}
