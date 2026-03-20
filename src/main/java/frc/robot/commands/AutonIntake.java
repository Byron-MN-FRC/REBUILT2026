package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hopper;

public class AutonIntake extends Command {
    private final Hopper m_hopper;

    public AutonIntake(Hopper hopper) {
        m_hopper = hopper;
        addRequirements(m_hopper);
    }

    @Override
    public void initialize() {
        // m_hopper.setHopperExtend();
    }

    @Override
    public void execute() {
        m_hopper.setFuelGrabberSpeed();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
