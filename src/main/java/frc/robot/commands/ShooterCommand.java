package frc.robot.commands;

import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
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
        shooter.setShooterSpeed(Constants.ShooterSubsystem.upperSpeed, Constants.ShooterSubsystem.lowerSpeed);
        

    }

@Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
    }
    @Override
    public boolean isFinished() {
        double speedError = Math.abs(int setShooterSpeed - upperSpeed, loserSpeed());
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
