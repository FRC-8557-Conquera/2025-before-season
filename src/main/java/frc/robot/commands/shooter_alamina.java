/*package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Peripheral;

public class shooter_alamina extends Command {
    private final Peripheral shooter;
    public shooter_alamina(Peripheral shooter){
        this.shooter = shooter;
        addRequirements(shooter);

    }
    @Override
    public void initialize(){

    }
    @Override
    public void execute(){
        shooter.shooterIcineal();
    }
    @Override
    public void end(boolean interrupt){
        shooter.shooterDurdur();
    }
    @Override
    public boolean isFinished(){
        
        return false;
    }
    
}
*/