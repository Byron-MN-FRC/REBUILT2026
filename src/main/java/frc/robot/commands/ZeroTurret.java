package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;

public class ZeroTurret extends Command {
    private final Turret m_turret;

    public ZeroTurret(Turret turret) {
        m_turret = turret;
        addRequirements(m_turret);
    }

     @Override
     public void initialize() {
         // Code to zero the turret goes here
     }

     @Override
     public void execute() {
         m_turret.checkZeroLeftFix(); // Example: Spin the turret at a low speed to find the zero position
     }

     @Override
     public boolean isFinished() {
         // Return true when the turret is successfully zeroed
         return true;
     }
}
