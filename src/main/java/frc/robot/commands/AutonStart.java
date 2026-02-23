package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.LedsSubsystem;

public class AutonStart extends SequentialCommandGroup {
    public AutonStart(Turret turret, ClimbSubsystem climb, LedsSubsystem leds) {
        addCommands(
            new ZeroAll(turret, climb, leds)
        );
    }
    
}
