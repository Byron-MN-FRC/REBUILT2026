package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.LedsSubsystem;

public class FuelGRAB extends Command {
    private final HopperSubsystem m_subsystem;
    private final LedsSubsystem m_leds;
    public FuelGRAB(HopperSubsystem subsystem, LedsSubsystem ledSubsystem) {
        m_subsystem = subsystem;
        m_leds = ledSubsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_subsystem);
    
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_leds.hopperRequestingLeds();
        if (m_leds.usingSubsystem == LedsSubsystem.SubsystemUsingLEDS.hopper) {
            m_leds.setColorOrange();
        }
        m_subsystem.isHopperExtendedDown();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_subsystem.setFuelGrabberSpeed();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_subsystem.isHopperExtendedUp();
        m_subsystem.stopFuelGrabber();
        m_leds.setColorNone();
        m_leds.noSubsystemUsingLeds();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}
