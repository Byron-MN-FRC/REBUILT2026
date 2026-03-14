package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hopper;

public class AutonIntakestop extends Command {
    private final Hopper m_hopper;
    private final Timer m_timer = new Timer();

    public AutonIntakestop(Hopper hopper) {
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
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}