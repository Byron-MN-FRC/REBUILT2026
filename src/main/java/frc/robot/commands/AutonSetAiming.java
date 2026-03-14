package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class AutonSetAiming extends Command {
    private double angle;
    private Turret m_turret;

    public AutonSetAiming(double angle) {
        this.angle = angle;
    }

    @Override
    public void initialize() {
        // This command doesn't require any subsystems, so we don't need to add any requirements.
        m_turret.aimDegrees(angle);
    }

    @Override
    public void execute() {
        // This command doesn't have any ongoing execution logic, so we can leave this empty.
    }

    @Override
    public void end(boolean interrupted) {
        // This command doesn't have any cleanup logic, so we can leave this empty.
    }

    @Override
    public boolean isFinished() {
        // This command should finish immediately after setting the aiming state.
        return true;
    }

}
