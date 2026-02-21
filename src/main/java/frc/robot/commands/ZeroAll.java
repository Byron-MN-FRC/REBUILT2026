package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.LedsSubsystem;
import frc.robot.subsystems.Turret;

public class ZeroAll extends SequentialCommandGroup {
    public ZeroAll(Turret turret, ClimbSubsystem climb, LedsSubsystem leds)  {
        addCommands(
            new ZeroTurret(turret),
            new ClimbZeroing(climb, leds)
        );
    }

}
