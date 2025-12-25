/*package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Peripheral;

public class shooter_duramina extends Command {
    private final Peripheral shooter;

    public shooter_duramina(Peripheral shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        // Komut başladığında shooter motorunu durdurur.
        shooter.shooterDurdur();
    }

    @Override
    public boolean isFinished() {
        // Komut, initialize() çağrıldıktan hemen sonra tamamlanır.
        return true;
    }
}
*/