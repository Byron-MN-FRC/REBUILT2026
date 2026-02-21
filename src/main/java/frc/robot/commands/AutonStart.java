package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Turret;

public class AutonStart extends SequentialCommandGroup {
    public AutonStart(Turret turret, Hopper hopper) {
        addCommands(
            new ZeroAll(turret, hopper)
        );
    }
    
}
