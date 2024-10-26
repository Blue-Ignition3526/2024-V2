package lib.BlueShift.control;

import java.util.Timer;
import java.util.TimerTask;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Notifier;

public class Buzzer {
    private final DigitalOutput buzzer;
    private Notifier toneNotifier;
    private boolean buzzerState;
    private double halfPeriod;

    public Buzzer(int channel) {
        this.buzzer = new DigitalOutput(channel);
    }

    public void playTone(double frequency, double duration) {
        halfPeriod = 1.0 / (2.0 * frequency);
        toneNotifier = new Notifier(() -> toggleBuzzer());

        toneNotifier.startPeriodic(halfPeriod);

        new Timer().schedule(new TimerTask() {
            @Override
            public void run() {
                stopTone();
            }
        }, (long)(duration * 1000));
    }

    private void toggleBuzzer() {
        buzzerState = !buzzerState;
        buzzer.set(buzzerState);
    }

    public void stopTone() {
        if (toneNotifier != null) toneNotifier.stop();
        buzzer.set(false);
    }
}
