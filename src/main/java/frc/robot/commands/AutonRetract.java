package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hopper;

public class AutonRetract extends Command {
    private final Hopper m_hopper;
    private final Timer m_timer = new Timer();


    public AutonRetract(Hopper hopper) {
        m_hopper = hopper;
        addRequirements(m_hopper);
    }

    @Override 
    public void initialize() {
        m_timer.reset();
        m_timer.start();
        m_hopper.setHopperRetract();
    }

    @Override
    public void execute() {
        // m_hopper.setFuelGrabberSpeed();
    }

    @Override
    public void end(boolean interrupted) {
        m_hopper.stopFuelGrabber();
        m_timer.stop();
        m_hopper.stopHopperFloorTransferSecure();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
