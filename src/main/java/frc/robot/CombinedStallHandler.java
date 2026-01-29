package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * CombinedStallHandler - Detects motor stalling by monitoring current draw and motion over time.
 * 
 * <p>Instantiate as a member variable in motor subsystems requiring stall detection.
 * Call {@link #isStalled()} from commands to check for stall conditions.</p>
 * <p><strong>Note:</strong> This class only works with TalonFX motors due to its reliance on
 * TalonFX-specific fault detection and velocity monitoring methods.</p>
 *
 * <p>In subsystem:</p>
 * <pre>{@code
 *      private CombinedStallHandler stallDetector = new CombinedStallHandler(motor);
 * }</pre>
 * In command: <pre>{@code if (stallDetector.isStalled()) { &#47;* handle stall *&#47; } </pre>
 * 
 * <p>Use cases: arm/elevator mechanisms, any system where stalling could cause damage.</p>
 */
public class CombinedStallHandler extends SubsystemBase {


    private TalonFX m_motor;
    private Timer m_stallTimer = new Timer();
    private double m_stallTime = 0.75; // Stall time in seconds
    private double stallingSpeed = 0.05; // Speed threshold for stalling

    /**
     * Creates a new CombinedStallHandler with default stall parameters.
     * 
     * @param motor The TalonFX motor to monitor for stalling conditions
     */
    public CombinedStallHandler(TalonFX motor) {
        this.m_motor = motor;
    }

    /**
     * Creates a new CombinedStallHandler with custom stall parameters.
     * 
     * @param motor The TalonFX motor to monitor for stalling conditions
     * @param stallTime Time in seconds that both stall conditions must be met before
     *                  reporting a stall (default: 0.75 seconds)
     * @param stallingSpeed Velocity threshold below which the motor is considered
     *                      stalled (default: 0.05 rotations per second)
     */
    public CombinedStallHandler(TalonFX motor, double stallTime, double stallingSpeed) {
        this.m_motor = motor;
        this.m_stallTime = stallTime;
        this.stallingSpeed = stallingSpeed;
    }

    /**
     * Periodic method that monitors stall conditions and manages the stall timer.
     * 
     * Checks current draw and velocity, starts timer when both conditions are met,
     * stops timer when either condition is no longer true.
     */
    @Override
    public void periodic() {
        boolean stallingCurrent = m_motor.getFault_StatorCurrLimit().getValue();
        boolean stallingMotion = Math.abs(m_motor.getVelocity().getValueAsDouble()) < stallingSpeed;

        if (m_stallTimer.isRunning()) {
            if (!stallingCurrent || !stallingMotion) {
                m_stallTimer.stop();
                m_stallTimer.reset();
            }
        } else {
            if (stallingCurrent && stallingMotion) {
                m_stallTimer.restart();
            }
        }
    }

    /**
     * Returns true if motor is stalled (both conditions met for configured duration).
     * 
     * @return true if stalled, false otherwise
     */
    public boolean isStalled() {
        return m_stallTimer.get() > m_stallTime; // try && isRunning() to avoid false positives
    }
}
