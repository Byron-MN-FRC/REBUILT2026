package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.HopperSubsystem;

public class AutonIntake extends Command {
    private final HopperSubsystem m_hopper;

    public AutonIntake(HopperSubsystem hopper) {
        m_hopper = hopper;
        addRequirements(m_hopper);
    }

    @Override
    public void initialize() {
        // m_hopper.setHopperExtend();
        m_hopper.setHopperExtendDown();
    }

    @Override
    public void execute() {
        m_hopper.setFuelGrabberSpeed(Constants.IntakeHopperConstants.FUEL_GRABBER_SPEED);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
