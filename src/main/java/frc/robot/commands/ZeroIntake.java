package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.HopperSubsystem;

public class ZeroIntake extends Command {
    private final HopperSubsystem m_hopper;

    public ZeroIntake(HopperSubsystem hopper) {
        m_hopper = hopper;
        addRequirements(m_hopper);
    }

     @Override
     public void initialize() {
         // Code to zero the turret goes here
  
     }

     @Override
     public void execute() {
         if (m_hopper.isHopperRetracted()) {
                m_hopper.setHopperRetract();
            } else {
                m_hopper.hopperZeroing();
            }
     }

     @Override
     public void end(boolean interrupted) {
        m_hopper.isHopperRetracted();
    }

     @Override
     public boolean isFinished() {
         // Return true when the turret is successfully zeroed
         return m_hopper.isHopperRetracted();
     }
}
