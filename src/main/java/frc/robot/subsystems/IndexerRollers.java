package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class IndexerRollers extends SubsystemBase {
    private final CANSparkMax rollersMotor; // declara el primer Spark de el primer motor de los rollers
    private final DigitalInput pieceSwitchIn; // declara el sensor de deteccion de nota primero
    private final DigitalInput pieceSwitchMiddle; // declara el sensor de deteccion de nota segundo

    private boolean isRolling = false; // variable de si rollers agarraron pieza empieza en falso

    private boolean pieceSwitchInEnabled = true; // abilita o usa el sensor, no lo ignora, empieza en true
    private boolean pieceSwitchMiddleEnabled = true; // abilita o usa el sensor, no lo ignora, empieza en true

    public IndexerRollers() { // metodo de construccion del subsistema
        this.rollersMotor = new CANSparkMax(Constants.Indexer.Rollers.kRollersMotorInID, MotorType.kBrushless); // pone el id y brushless del primer motor de los rollers
        this.pieceSwitchIn = new DigitalInput(Constants.Indexer.Rollers.kpieceSwitchInPort); // el puerto de roborio del sensor de deteccion de nota primero
        this.pieceSwitchMiddle = new DigitalInput(Constants.Indexer.Rollers.kpieceSwitchMiddlePort); // el puerto de roborio del sensor de deteccion de nota segundo
        rollersMotor.setSmartCurrentLimit(Constants.Indexer.Rollers.krollersCurrentLimiterInAmps);
    }

    // * Idle modes (for not damaging gearbox)
    public void setRollersCoast() { // metodo que pone los rollers en brake para que los motores frenen en seco
        this.rollersMotor.setIdleMode(CANSparkMax.IdleMode.kCoast); // establece el modo ideal de los primeros rollers en brake
        SmartDashboard.putBoolean("IndexIn/Brake", false); // en la dashboard aparece si se activa el modo brake
    }

    public void setRollersBrake() { // metodo que pone los rollers en brake para que los motores frenen en seco
        SmartDashboard.putBoolean("IndexIn/Brake", true); // en la dashboard aparece si se activa el modo brake
        this.rollersMotor.setIdleMode(CANSparkMax.IdleMode.kBrake); // establece el modo ideal de los primeros rollers en brake
    }

    public void setPieceSwitchInEnabled(boolean enabled) { // metodo para establecer que abilitas o usas el sensor, no lo ignoras,
        this.pieceSwitchInEnabled = enabled; // usas el primer sensor, no lo ignoras
    }

    public void setPieceSwitchMiddleEnabled(boolean enabled) { // metodo para establecer que abilitas o usas el sensor, no lo ignoras,
        this.pieceSwitchMiddleEnabled = enabled; // usas el segundo sensor, no lo ignoras
    }

    // * Piece switch for in index
    public boolean hasPieceIn() { // metodo para cuando tenga la pieza
        return !this.pieceSwitchIn.get() && pieceSwitchInEnabled; // regresa lo contrario del valor que el sensor te de (que es si el intake tiene la pieza en base a lo que te dice el sensor) y abilitas o tomas en cuenta el sensor del in, no lo ignoras
    }

    // * Piece switch for middle index
    public boolean hasPieceMiddle() { // metodo para cuando tenga la pieza
        return !this.pieceSwitchMiddle.get() && pieceSwitchMiddleEnabled; // regresa lo contrario del valor que el sensor te de (que es si el intake tiene la pieza en base a lo que te dice el sensor) y abilitas o tomas en cuenta el sensor del middle, no lo ignoras
    }

     public boolean hasPieceRetained() { // metodo para cuando tenga la pieza en medio
        return hasPieceIn() && hasPieceMiddle(); // regresa si tiene la pieza detectada por los dos sensores
    }

    // * Speed setters
    public void setRollersSpeed(double speed) { // metodo para establecer la velocidad de los primeros rollers
        this.isRolling = speed > 0; // si la velocidad es positiva significa que esta agarrando pieza
        this.rollersMotor.set(speed); // establece la velocidad de los rollers
        SmartDashboard.putBoolean("IndexIn/Receiving", this.isRolling); // en la dashboard aparece si esta agarrando la pieza
    }

    public void stop() { // metodo para establecer la velocidad cuando los rollers se paran
        this.setRollersSpeed(0); // establece la velocidad en 0
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("RollersIn/Indexing", this.hasPieceIn()); // aparece en la dashboard si el index in tiene la pieza
        SmartDashboard.putBoolean("RollersMiddle/Indexing", this.hasPieceMiddle()); // aparece en la dashboard si el index middle tiene la pieza
    }
}
