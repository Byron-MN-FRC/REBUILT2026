

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.LedsSubsystem;

public class ClimbZeroing extends Command {
    private final ClimbSubsystem m_climb;
    private final LedsSubsystem m_leds;
    public ClimbZeroing(ClimbSubsystem subsystem, LedsSubsystem ledSubsystem)  {
        m_climb = subsystem;
        m_leds = ledSubsystem;
        addRequirements(m_climb);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (Constants.Debug.DEBUG_MODE) {
            System.out.println("+---------------+");
            System.out.println("| Climb Zeroing |");
            System.out.println("+---------------+");
        }
        m_leds.setModeBlueFlashing();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_climb.setClimbZeroing();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_climb.setBottom();
        m_climb.setRaiserPosition(Constants.ClimbConstants.RAISER_LOWER_TARGET);
        m_climb.climbStageReset();
        if (Constants.Debug.DEBUG_MODE) {
            System.out.println("+--------------+");
            System.out.println("| Climb Zeroed |");
            System.out.println("+--------------+");
            System.out.println("=");
            System.out.println("Disabling lights...");
        }
        m_leds.setColorNone();

        m_climb.stopClimbing();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_climb.getBottomSwitch();
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}
