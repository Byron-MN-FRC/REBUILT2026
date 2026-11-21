package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.LedsSubsystem;

public class AutonExtend extends Command {
    private final HopperSubsystem m_hopper;
    private final LedsSubsystem m_leds;


    public AutonExtend(HopperSubsystem hopper, LedsSubsystem m_leds) {
        m_hopper = hopper;
        addRequirements(m_hopper);
        this.m_leds = m_leds;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_hopper.setHopperExtendUp();
        m_leds.hopperRequestingLeds();
        if (m_leds.usingSubsystem == LedsSubsystem.SubsystemUsingLEDS.hopper) {
            m_leds.setColorWhite();
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_hopper.setHopperFloorTransferSecureSpeed(Constants.IntakeHopperConstants.HOPPER_FLOOR_TRANSFER_SECURE_SPEED);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_leds.setColorNone();
        m_leds.noSubsystemUsingLeds();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_hopper.isHopperExtendedUp();
    }

}
