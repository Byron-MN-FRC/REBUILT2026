package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Turret;

public class ZeroAll extends SequentialCommandGroup {
    public ZeroAll(Turret turret, Hopper hopper) {
        addCommands(
            new ZeroTurret(turret),
            new ZeroHopper(hopper)
        );
    }

}
