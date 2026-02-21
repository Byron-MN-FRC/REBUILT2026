package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.Turret;

public class AutonStart extends SequentialCommandGroup {
    public AutonStart(Turret turret, ClimbSubsystem climb) {
        addCommands(
            new ZeroAll(turret, climb)
        );
    }
    
}
