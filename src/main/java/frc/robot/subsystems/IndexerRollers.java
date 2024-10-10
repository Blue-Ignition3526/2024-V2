package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class IndexerRollers extends SubsystemBase {
    private final CANSparkMax rollersMotorIn; // declara el primer Spark de el primer motor de los rollers
    private final CANSparkMax rollersMotorOut; // declara el segundo Spark de el segundo motor de los rollers
    private final DigitalInput pieceSwitchIn; // declara el sensor de deteccion de nota primero
    private final DigitalInput pieceSwitchMiddle; // declara el sensor de deteccion de nota segundo

    private boolean isRollingIn = false; // variable de si rollers agarraron pieza empieza en falso
    private boolean isRollingMiddle = false; // variable de si la pieza esta en medio de los dos rollers empieza en falso

    private boolean pieceSwitchInEnabled = true; // abilita o usa el sensor, no lo ignora, empieza en true
    private boolean pieceSwitchMiddleEnabled = true; // abilita o usa el sensor, no lo ignora, empieza en true

    public IndexerRollers() { // metodo de construccion del subsistema
        this.rollersMotorIn = new CANSparkMax(Constants.Indexer.Rollers.kRollersMotorInID, MotorType.kBrushless); // pone el id y brushless del primer motor de los rollers
        this.rollersMotorOut = new CANSparkMax(Constants.Indexer.Rollers.kRollersMotorOutID, MotorType.kBrushless); // pone el id y brushless del segundo motor de los rollers
        this.pieceSwitchIn = new DigitalInput(Constants.Indexer.Rollers.kpieceSwitchInPort); // el puerto de roborio del sensor de deteccion de nota primero
        this.pieceSwitchMiddle = new DigitalInput(Constants.Indexer.Rollers.kpieceSwitchMiddlePort); // el puerto de roborio del sensor de deteccion de nota segundo
    }

    // * Idle modes (for not damaging gearbox)
    public void setRollersCoast() { // metodo que pone los rollers en coast para cuidar la gearbox, cuando el motor deje de recibir energia no se va a parar va a sefuir avanzando hasta que se detenga por friccion
        this.setRollersInCoast(); // rollers in en coast
        this.setRollersOutCoast(); // rollers in en coast
    }

    public void setRollersInCoast() { // metodo que pone los rollers en brake para que los motores frenen en seco
        this.rollersMotorIn.setIdleMode(CANSparkMax.IdleMode.kCoast); // establece el modo ideal de los primeros rollers en brake
        SmartDashboard.putBoolean("IndexIn/Brake", false); // en la dashboard aparece si se activa el modo brake
    }

    public void setRollersOutCoast() { // metodo que pone los rollers en brake para que los motores frenen en seco
        this.rollersMotorOut.setIdleMode(CANSparkMax.IdleMode.kCoast); // establece el modo ideal de los primeros rollers en brake
        SmartDashboard.putBoolean("IndexOut/Brake", false); // en la dashboard aparece si se activa el modo brake
    }

    public void setRollersInBrake() { // metodo que pone los rollers en brake para que los motores frenen en seco
        SmartDashboard.putBoolean("IndexIn/Brake", true); // en la dashboard aparece si se activa el modo brake
        this.rollersMotorIn.setIdleMode(CANSparkMax.IdleMode.kBrake); // establece el modo ideal de los primeros rollers en brake
    }

    public void setRollersOutBrake() {
        SmartDashboard.putBoolean("IndexOut/Brake", true); // en la dashboard aparece si se activa el modo brake
        this.rollersMotorOut.setIdleMode(CANSparkMax.IdleMode.kBrake); // establece el modo ideal de los segundos rollers en brake
    }

    public void setRollersBrake() { // pone en brake ambos rollers
        this.setRollersInBrake(); // roller in en brake
        this.setRollersOutBrake(); // roller in en brake
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

    public void setPieceSwitchInEnabled(boolean enabled) { // metodo para establecer que abilitas o usas el sensor, no lo ignoras,
        this.pieceSwitchInEnabled = enabled; // usas el primer sensor, no lo ignoras
    }

    public void setPieceSwitchMiddleEnabled(boolean enabled) { // metodo para establecer que abilitas o usas el sensor, no lo ignoras,
        this.pieceSwitchMiddleEnabled = enabled; // usas el segundo sensor, no lo ignoras
    }

    // * Speed setters
    public void setRollersInSpeed(double speed) { // metodo para establecer la velocidad de los primeros rollers
        this.isRollingIn = speed > 0; // si la velocidad es positiva significa que esta agarrando pieza
        this.rollersMotorIn.set(speed); // establece la velocidad de los rollers
        SmartDashboard.putBoolean("IndexIn/Receiving", this.isRollingIn); // en la dashboard aparece si esta agarrando la pieza
    }

    public void setRollersInReceivingSpeed() {
        this.setRollersInSpeed(Constants.Indexer.Rollers.krollersInReceivingSpeed); // establece la velocidad de los primeros rollers cuando reiben
    }

    public void setRollersInExpulsingSpeed() { // metodo para establecer la velocidad cuando los primeros rollers quieren expulsar
        this.setRollersInSpeed(Constants.Indexer.Rollers.krollersInExpulsingSpeed); // establece la velocidad de los primeros rollers cuando expulsan
    }

    public void setRollersInPassSpeed() { // metodo para establecer la velocidad cuando los primeros rollers quieren pasar hacia los out rollers
        this.setRollersInSpeed(Constants.Indexer.Rollers.krollersInPassSpeed); // establece la velocidad de los primeros rollers quieren pasar hacia los out rollers
    }

    public void setRollersInRetainSpeed() { // metodo para establecer la velocidad cuando los primeros rollers quieren pasar hacia los out rollers
        this.setRollersInSpeed(Constants.Indexer.Rollers.krollersInRetainSpeed); // establece la velocidad de los primeros rollers quieren pasar hacia los out rollers
    }

    public void setRollersInHold() { // metodo hold en in
        this.setRollersInSpeed(Constants.Indexer.Rollers.krollersInHoldSpeed); //velocidad de los rollers in para rnotaetener
    }

    public void setRollersOutSpeed(double speed) { // metodo para establecer la velocidad de los segundos rollers
        this.isRollingMiddle = speed > 0; // si la velocidad es positiva significa que esta agarrando pieza
        this.rollersMotorOut.set(speed); // establece la velocidad de los rollers
        SmartDashboard.putBoolean("IndexMiddle/Receiving", this.isRollingMiddle); // en la dashboard aparece si esta agarrando pieza
    }

    public void setRollersOutReceivingSpeed() {
        this.setRollersOutSpeed(Constants.Indexer.Rollers.krollersOutReceivingSpeed); // establece la velocidad de los primeros rollers cuando reciben
    }

    public void setRollersOutExpulsingSpeed() { // metodo para establecer la velocidad cuando los primeros rollers quieren expulsar
        this.setRollersOutSpeed(Constants.Indexer.Rollers.krollersOutExpulsingSpeed); // establece la velocidad de los segundos rollers cuando expulsan
    }

    public void setRollersOutPassSpeed() { // metodo para establecer la velocidad cuando los out rollers quieren pasar hacia el shooter
        this.setRollersInSpeed(Constants.Indexer.Rollers.krollersOutPassSpeed); // establece la velocidad de los out rollers quieren pasar hacia el shooter
    }

     public void setRollersOutRetainSpeed() { // metodo para establecer la velocidad cuando los primeros rollers quieren pasar hacia los out rollers
        this.setRollersInSpeed(Constants.Indexer.Rollers.krollersOutRetainSpeed); // establece la velocidad de los primeros rollers quieren pasar hacia los out rollers
    }

    public void setRollersOutHold() { // metodo hold en out
        this.setRollersOutSpeed(Constants.Indexer.Rollers.krollersOutHoldSpeed); //velocidad de los rollers out para rnotaetener 
    }

     public void indexingFull() { // todos los roller recibiendo
        this.setRollersInReceivingSpeed(); // rollers in recibir
        this.setRollersOutReceivingSpeed(); // rollers out recibir
    }

    public void outakingFull() { // todos los roller expulsando
        this.setRollersInExpulsingSpeed(); // roller in expulsa
        this.setRollersOutExpulsingSpeed(); // roller out expulsa
    }

    public void setRollersHold() { // metodo para que todos los rollers esten en modo retener la nota
        this.setRollersInHold(); // establece la velocidad de los rollers cuando estan en posicion de retener la nota
        this.setRollersOutHold(); // establece la velocidad de los rollers cuando estan en posicion de retener la nota
    }

    public void stop() { // metodo para establecer la velocidad cuando los rollers se paran
        this.setRollersInSpeed(0); // establece la velocidad en 0
        this.setRollersOutSpeed(0); // establece la velocidad en 0
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("RollersIn/Indexing", this.hasPieceIn()); // aparece en la dashboard si el index in tiene la pieza
        SmartDashboard.putBoolean("RollersMiddle/Indexing", this.hasPieceMiddle()); // aparece en la dashboard si el index middle tiene la pieza
    }
}
