package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.LedsSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.Turret;

public class ZeroAll extends ParallelCommandGroup {
    public ZeroAll(Turret turret, ClimbSubsystem climb, LedsSubsystem leds, HopperSubsystem hopper)  {
        addCommands(
            new ZeroTurret(turret),
            new ZeroIntake(hopper),
            new ClimbZeroing(climb, leds)
        );
    }
}
