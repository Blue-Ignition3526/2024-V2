package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
// lower y upper rollers



public class ShooterCommand extends Command {
    private final ShooterSubsystem shooter;
    // private final DistanceSensor distanceSensor;

    /*public ShooterCommand(ShooterSubsystem shooter, Limelight limelight) { //DUDA
        this.shooter = shooter;
        // this.distanceSensor = distanceSensor;
        addRequirements(shooter);
    }*/

    public ShooterCommand(ShooterSubsystem shooter){
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize(){
    
    }

@Override
    public void execute() {
     //   double distance = limelight.getDistance(); 
      //  double upperSpeed = calculateUpperSpeed(distance);
      //  double lowerSpeed = calculateLowerSpeed(distance);
      //  shooter.shoot(upperSpeed, lowerSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }
    @Override
    public boolean isFinished() {
        return false; // Continúa hasta que se interrumpa
    }
    //TODO: mover calculates
    //TODO: cambiar nombres
    //TODO: calculos con la tabla
    private double calculateUpperSpeed(double distance) {
        // Ajusta esta lógica según las necesidades 
        if (distance < 1.0) return 0.5;
        else if (distance < 2.0) return 0.7;
        else return 1.0; // depende de cambios
    }


    private double calculateLowerSpeed(double distance) {
        // Ajusta esta lógica según las necesidades 
        return calculateUpperSpeed(distance); // Ejemplo
    }
}
