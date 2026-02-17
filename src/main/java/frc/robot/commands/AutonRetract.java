package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hopper;

public class AutonRetract extends Command {
    private final Hopper m_hopper;

    public AutonRetract(Hopper hopper) {
        m_hopper = hopper;
        addRequirements(m_hopper);
    }

    @Override 
    public void initialize() {

        m_hopper.setHopperRetract();
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return m_hopper.isHopperRetracted();
    }

}
