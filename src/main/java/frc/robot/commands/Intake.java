

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.HopperSubsystem.HopperState;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.LedsSubsystem;

public class Intake extends Command {
    private final HopperSubsystem m_hopper;
    private final Turret m_turret;
    private final LedsSubsystem m_leds;

    public Intake(HopperSubsystem hopperSubsystem, Turret turretSubsystem, LedsSubsystem ledSubsystem)  {
        this(hopperSubsystem, turretSubsystem, ledSubsystem, true);
    }

    public Intake(HopperSubsystem hopperSubsystem, Turret turretSubsystem, LedsSubsystem ledSubsystem, boolean requireTurret)  {
        m_hopper = hopperSubsystem;
        m_turret = turretSubsystem;
        m_leds = ledSubsystem;
        if (requireTurret) {
            addRequirements(m_turret);
        }
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_leds.hopperRequestingLeds();
        if (m_hopper.targetHopperState == HopperState.retracted) {
            if (m_leds.usingSubsystem == LedsSubsystem.SubsystemUsingLEDS.hopper) {
                m_leds.setModeGreenFlashing();
            }
            m_hopper.setHopperExtendUp();
            m_turret.aimDegrees(Constants.TurretShooterConstants.NEUTRAL_POSITION);
        }
        else {
            if (m_leds.usingSubsystem == LedsSubsystem.SubsystemUsingLEDS.hopper) {
                m_leds.setColorLightBlue();
            }
            m_hopper.setHopperRetract();
        }
        if (Constants.Debug.DEBUG_MODE) System.out.println("Lights on (hopper)");
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_leds.setColorNone();
        m_leds.noSubsystemUsingLeds();

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (m_hopper.targetHopperState == HopperState.extendedUp) {
            return m_hopper.isHopperExtendedUp();
        } else if (m_hopper.targetHopperState == HopperState.retracted) {
            return m_hopper.isHopperRetracted() || m_hopper.getHopperRetractSwitch();
        } else {
            return m_hopper.isHopperExtendedUp();
        }
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;

    }
}
