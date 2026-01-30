package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.climb.LockdownMode;
public class dummy extends SubsystemBase {
    public dummy() { }
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if (RobotContainer.getInstance().m_climb.currentLockdownMode == LockdownMode.partial) { //there is only a difference between partial and full if you are the wheels because the wheels need to move in partial but not full
            // Apply partial lockdown behavior
        } else if (RobotContainer.getInstance().m_climb.currentLockdownMode == LockdownMode.full) {
            // Apply full lockdown behavior
        } else {
            // No lockdown behavior
        }
    }
}