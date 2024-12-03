package lib.BlueShift.control;

import java.util.Timer;
import java.util.TimerTask;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.PWM;

public class Buzzer {
    private final PWM buzzer;
    private Notifier toneNotifier;
    private boolean buzzerState;
    private double halfPeriod;

    public Buzzer(int channel) {
        this.buzzer = new PWM(channel);
        buzzer.enableDeadbandElimination(true);
    }

    public void playTone(double frequency) {
        halfPeriod = 1.0 / (2.0 * frequency);
        toneNotifier = new Notifier(() -> toggleBuzzer());
        toneNotifier.startPeriodic(halfPeriod);
    }

    public void playTone(double frequency, double duration) {
        playTone(frequency);
        new Timer().schedule(new TimerTask() {
            @Override
            public void run() {
                stopTone();
            }
        }, (long) (duration * 1000));
    }

    private void toggleBuzzer() {
        buzzerState = !buzzerState;
        buzzer.setPosition(buzzerState ? 4096 : 0);
    }

    public void stopTone() {
        if (toneNotifier != null) {
            toneNotifier.stop();
            toneNotifier.close();
        }
        buzzer.setPosition(0);
    }
}
