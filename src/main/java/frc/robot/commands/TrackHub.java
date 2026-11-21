package frc.robot.commands;

import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.TurretCam;
import frc.robot.subsystems.LedsSubsystem;
import frc.robot.subsystems.Turret;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TrackHub extends Command {

    private Turret m_turret;
    private final LedsSubsystem m_leds;
    private Timer m_timer = new Timer();

    private final TimeInterpolatableBuffer<Double> m_turretAngleHistory = TimeInterpolatableBuffer.createDoubleBuffer(0.5);
    private double m_lastHeartbeat = -1;

    /** Creates a new trackHub. */
    public TrackHub(Turret subsystem, LedsSubsystem leds) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
        m_leds = leds;
        m_turret = subsystem;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_leds.turretRequestingLeds();
        m_timer.reset();
        m_timer.stop();

        m_turretAngleHistory.clear();
        m_lastHeartbeat = -1;

        int[] hubTags = Constants.DriveConstants.getAlliance() == Alliance.Red
                ? Constants.FieldConstants.RED_HUB_CENTER_TAGS
                : Constants.FieldConstants.BLUE_HUB_CENTER_TAGS;
        LimelightHelpers.SetFiducialIDFiltersOverride(Constants.VisionConstants.TURRET_CAM, hubTags);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_leds.turretRequestingLeds();
        double now = Timer.getFPGATimestamp();
        m_turretAngleHistory.addSample(now, m_turret.getAngleDegrees());

        if (TurretCam.targetLocated() == true) {
            double heartbeat = LimelightHelpers.getHeartbeat(Constants.VisionConstants.TURRET_CAM);
            if (heartbeat != m_lastHeartbeat) {
                m_lastHeartbeat = heartbeat;

                double latencySeconds = (LimelightHelpers.getLatency_Pipeline(Constants.VisionConstants.TURRET_CAM)
                        + LimelightHelpers.getLatency_Capture(Constants.VisionConstants.TURRET_CAM)) / 1000.0;
                double angleAtCapture = m_turretAngleHistory.getSample(now - latencySeconds)
                        .orElse(m_turret.getAngleDegrees());

                m_turret.aimDegrees(angleAtCapture + TurretCam.getAngleError());
            }
            m_timer.stop();
            m_timer.reset();

        } else if (!TurretCam.targetLocated() && !m_timer.isRunning()) {
            m_timer.start();
        }

        if (m_timer.hasElapsed(Constants.TurretShooterConstants.TURRET_CAM_TIMEOUT)) {
            m_turret.aimDegrees(Constants.TurretShooterConstants.NEUTRAL_POSITION);
        }

        // Generate LED colors based on how close the turret angle is to the target.
        // Green is on target and red is not.
        if (Math.abs(TurretCam.getAngleError()) <= 2.0 && TurretCam.targetLocated() == true) { //should trigger when target located and the error is less the 0.1
            if (m_leds.usingSubsystem == LedsSubsystem.SubsystemUsingLEDS.turret) {
                m_leds.setColorGreen();
            }
        } else {
            if (m_leds.usingSubsystem == LedsSubsystem.SubsystemUsingLEDS.turret) {
                m_leds.setColorRed();
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_turret.aimDegrees(0);
        m_leds.noSubsystemUsingLeds();
        LimelightHelpers.SetFiducialIDFiltersOverride(Constants.VisionConstants.TURRET_CAM,
                Constants.FieldConstants.TAGS_FOR_POSE_ESTIMATION);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
