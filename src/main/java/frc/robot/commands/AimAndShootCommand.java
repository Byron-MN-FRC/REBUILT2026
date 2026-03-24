package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.LedsSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

/**
 * Combined aim-and-shoot command group.
 * <p>
 * Runs two commands in parallel:
 * <ul>
 *   <li>{@link AimAtTarget} — uses the turret-mounted limelight to detect AprilTags,
 *       rotates the turret to aim at the computed target point, and publishes
 *       "Targeting/Target RPM" based on distance.</li>
 *   <li>{@link TargetingShootCommand} — reads the published RPM each cycle,
 *       spins up the shooter, and feeds balls once at speed.</li>
 * </ul>
 * These use different subsystems (Turret vs Shooter) so they run without conflict.
 * <p>
 * This follows the same pattern as {@link SnowBlowerCommandGroup}.
 */
public class AimAndShootCommand extends ParallelCommandGroup {

    public AimAndShootCommand(Turret turret, Shooter shooter, HopperSubsystem hopper, LedsSubsystem leds) {
        addCommands(
            new AimAtTarget(turret),
            new TargetingShootCommand(shooter, hopper, leds)
        );
    }
}
