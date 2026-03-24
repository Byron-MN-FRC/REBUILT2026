// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.LedsSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SnowBlowerCommandGroup extends ParallelCommandGroup {
  /** Creates a new SnowBlowerCommandGroup. */
  public SnowBlowerCommandGroup(double rpm, Shooter shooterSubsystem, HopperSubsystem hopperSubsystem,
      LedsSubsystem ledSubsystem, Turret m_turret) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new RPMShootCommand(
            rpm,
            shooterSubsystem,
            hopperSubsystem,
            ledSubsystem),
        new SnowBlowerAiming(
            m_turret));
  }
}
