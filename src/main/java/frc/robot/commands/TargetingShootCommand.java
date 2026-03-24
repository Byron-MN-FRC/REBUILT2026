package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.LedsSubsystem;
import frc.robot.subsystems.Shooter;

/**
 * Shoot command that reads its target RPM from SmartDashboard every cycle.
 * Designed to run in parallel with {@link AimAtTarget}, which publishes
 * "Targeting/Target RPM" based on limelight distance.
 * <p>
 * Gate and magazine logic is identical to {@link RPMShootCommand}: the shooter
 * spins up to the target RPM, and once it reaches speed the gate and magazine
 * feed balls into the shooter.
 */
public class TargetingShootCommand extends Command {

    private static final String RPM_KEY = "Targeting/Target RPM";

    private final Shooter m_shooter;
    private final HopperSubsystem m_hopper;
    private final LedsSubsystem m_leds;
    private final Timer m_timer = new Timer();

    private final double agitateForwardTime = 0.75;
    private final double agitateReverseTime = 0;

    public TargetingShootCommand(Shooter shooter, HopperSubsystem hopper, LedsSubsystem leds) {
        m_shooter = shooter;
        m_hopper = hopper;
        m_leds = leds;
        addRequirements(m_shooter);
    }

    @Override
    public void initialize() {
        m_timer.reset();

        m_leds.shooterRequestingLeds();
        if (m_leds.usingSubsystem == LedsSubsystem.SubsystemUsingLEDS.shooter) {
            m_leds.setColorRed();
        }
    }

    @Override
    public void execute() {
        // Read the RPM that AimAtTarget is publishing each frame
        double rpm = SmartDashboard.getNumber(RPM_KEY, 0);
        m_shooter.setTargetRPM(rpm);
        m_shooter.spinShooter(rpm);

        m_hopper.setFuelGrabberSpeed();

        if (m_shooter.isAtTargetRPM()) {
            // Shooter is at speed — feed balls
            if (m_timer.isRunning()) {
                m_timer.stop();
            }
            m_shooter.runGate(Constants.TurretShooterConstants.GATE_FORWARD_SPEED);
            m_shooter.runMagazine(Constants.TurretShooterConstants.MAGAZINE_FORWARD_SPEED);
            m_hopper.setHopperFloorTransferSecureSpeed(
                    Constants.IntakeHopperConstants.HOPPER_FLOOR_TRANSFER_SECURE_SPEED);

            // LED green when shooting
            if (m_leds.usingSubsystem == LedsSubsystem.SubsystemUsingLEDS.shooter) {
                m_leds.setColorGreen();
            }
        } else {
            // Still spinning up — agitate to prevent jams
            if (!m_timer.isRunning() || m_timer.hasElapsed(agitateForwardTime + agitateReverseTime)) {
                m_timer.restart();
            }
            m_shooter.stopMagazine();

            // LED red while spinning up
            if (m_leds.usingSubsystem == LedsSubsystem.SubsystemUsingLEDS.shooter) {
                m_leds.setColorRed();
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
        m_hopper.stopAll();
        m_shooter.stopAll();

        m_leds.setColorNone();
        m_leds.noSubsystemUsingLeds();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}
