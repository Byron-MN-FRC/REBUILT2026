package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Turret;

public class SnowBlowerAiming extends Command {
    private double angle;
    private Turret m_turret;

    public SnowBlowerAiming(Turret m_turret) {
        this.m_turret = m_turret;
        this.angle = Constants.DriveConstants.getAlliance() == Alliance.Red ? 0 : 180;
        addRequirements(m_turret);
    }

    @Override
    public void initialize() {
        // This command doesn't require any subsystems, so we don't need to add any requirements.
    }
    
    @Override
    public void execute() {
        // This command doesn't have any ongoing execution logic, so we can leave this empty.
        m_turret.aimFieldRelativeAngle(angle);
    }

    @Override
    public void end(boolean interrupted) {
        // This command doesn't have any cleanup logic, so we can leave this empty.
        m_turret.aimDegrees(Constants.TurretShooterConstants.NEUTRAL_POSITION);
    }

    @Override
    public boolean isFinished() {
        // This command should finish immediately after setting the aiming state.
        return false;
    }

}
