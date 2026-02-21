package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class ZeroClimb extends Command {
    private final ClimbSubsystem m_climb ;

    public ZeroClimb(ClimbSubsystem climb) {
        m_climb = climb;
        addRequirements(m_climb);
    }

     @Override
     public void initialize() {
         // Code to zero the hopper goes here
     }
@Override
public void execute() {
    m_climb.setArmZeroing(); // Example: Stop all climb motors to find the zero position
}
     @Override
     public boolean isFinished() {
         // Return true when the hopper is successfully zeroed
         return true;
     }

}
