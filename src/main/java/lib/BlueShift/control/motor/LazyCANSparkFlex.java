package lib.BlueShift.control.motor;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.REVLibError;

/**
 * A wrapper for CANSparkMax that only updates the controller if the value has changed.
 */
public class LazyCANSparkFlex extends CANSparkFlex {
    public LazyCANSparkFlex(int deviceID, MotorType type, boolean lazy) {
        super(deviceID, type);

        if (lazy) {
            setPeriodicFramePeriod(PeriodicFrame.kStatus0, 15);
            setPeriodicFramePeriod(PeriodicFrame.kStatus1, 30);
            setPeriodicFramePeriod(PeriodicFrame.kStatus2, 50);
        }
    }

    public LazyCANSparkFlex(int deviceID, MotorType type) {
        this(deviceID, type, true);
    }

    public void set(double value) {
        if (value != get()) super.set(value);
    }

    public void setInverted(boolean isInverted) {
        if (isInverted != getInverted()) super.setInverted(isInverted);
    }

    public REVLibError setClosedLoopRampRate(double seconds) {
        if (seconds != getClosedLoopRampRate()) return super.setClosedLoopRampRate(seconds);
        return REVLibError.kOk;
    }

    public REVLibError setOpenLoopRampRate(double seconds) {
        if (seconds != getOpenLoopRampRate()) return super.setOpenLoopRampRate(seconds);
        return REVLibError.kOk;
    }
}
