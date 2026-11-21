package frc.robot.commands.ledtestcommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LedsSubsystem;

public class fasterfaster extends Command {

    private final LedsSubsystem m_leds;

    public fasterfaster( LedsSubsystem ledSubsystem)  {
        m_leds = ledSubsystem;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_leds.setModeFasterFaster();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_leds.setColorNone();
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
