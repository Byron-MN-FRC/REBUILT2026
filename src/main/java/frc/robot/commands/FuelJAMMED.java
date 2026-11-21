

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.Shooter;

public class FuelJAMMED extends Command {
    private final HopperSubsystem m_hopper;
    private final Shooter m_shooter;
    public FuelJAMMED(HopperSubsystem m_hopper, Shooter m_shooter) {
        this.m_hopper = m_hopper;
        this.m_shooter = m_shooter;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(m_hopper, m_shooter);


    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_hopper.ifIntakeJammed();
        m_shooter.runMagazine(Constants.TurretShooterConstants.JAM_CLEAR_SPEED);
        m_shooter.runGate(Constants.TurretShooterConstants.JAM_CLEAR_SPEED);
        m_shooter.runPreFylWheel(Constants.TurretShooterConstants.JAM_CLEAR_SPEED);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_hopper.stopFuelGrabber();
        m_shooter.stopGates();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}
