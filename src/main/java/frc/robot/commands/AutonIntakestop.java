package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HopperSubsystem;

public class AutonIntakestop extends Command {
    private final HopperSubsystem m_hopper;

    public AutonIntakestop(HopperSubsystem hopper) {
        m_hopper = hopper;
        addRequirements(m_hopper);
    }

    @Override
    public void initialize() {
        // m_hopper.setHopperExtend();;
    }

    @Override
    public void end(boolean interrupted) {
        m_hopper.stopFuelGrabber();
        m_hopper.isHopperExtendedUp();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
