package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
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
         if (m_turret.zeroSwitch.get()) {
                m_turret.rotateShooterMotor.setPosition(Constants.TurretShooterConstants.MAX_LEFT_POSITION);
            } else {
                m_turret.rotateShooterMotor.set(-0.04);
            }
     }

     @Override
     public void end(boolean interrupted) {
        m_turret.aimDegrees(Constants.TurretShooterConstants.NEUTRAL_POSITION);
    }

     @Override
     public boolean isFinished() {
         // Return true when the turret is successfully zeroed
         return m_turret.zeroSwitch.get();
     }
}
