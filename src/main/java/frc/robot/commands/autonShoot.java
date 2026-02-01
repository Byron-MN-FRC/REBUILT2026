package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hopper;


//All temporary stuff, fix when shooting comman is made
public class autonShoot extends Command {
    private final Hopper m_hopper;
    private final Timer m_timer = new Timer();
    private final double m_duration;

    public autonShoot(Hopper hopper, double duration) {
        m_hopper = hopper;
        m_duration = duration;
        addRequirements(m_hopper);
    }

    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();
        m_hopper.setHopperExtend();
    }

    @Override
    public void execute() {
        m_hopper.setFuelGrabberSpeed();
    }

    @Override
    public void end(boolean interrupted) {
        m_hopper.stopFuelGrabber();
        m_timer.stop();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
