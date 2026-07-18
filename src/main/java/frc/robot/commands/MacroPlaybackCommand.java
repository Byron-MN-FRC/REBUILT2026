package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.MacroManager;
import frc.robot.MacroManager.MacroFrame;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.HopperSubsystem.HopperState;
import frc.robot.subsystems.LedsSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class MacroPlaybackCommand extends Command {
    private final CommandSwerveDrivetrain m_drivetrain;
    private final HopperSubsystem m_hopper;
    private final Shooter m_shooter;
    private final Turret m_turret;
    private final LedsSubsystem m_leds;

    private List<MacroFrame> m_frames = new ArrayList<>();
    private final Timer m_timer = new Timer();

    private final SwerveRequest.FieldCentric m_driveRequest = new SwerveRequest.FieldCentric()
        .withDriveRequestType(com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType.OpenLoopVoltage);

    // Instances of commands to schedule/cancel during playback
    private Command m_intakeCommand;
    private Command m_shootCommand;
    private Command m_fuelGrabCommand;
    private Command m_agitateCommand;
    private Command m_fuelJammedCommand;
    private Command m_zeroTurretCommand;
    private Command m_zeroIntakeCommand;

    // Previous states for edge detection
    private boolean prevB = false;
    private boolean prevY = false;
    private boolean prevLB = false;
    private boolean prevRB = false;
    private boolean prevLT = false;
    private boolean prevRT = false;
    private boolean prevBack = false;

    public MacroPlaybackCommand(CommandSwerveDrivetrain drivetrain, HopperSubsystem hopper,
                                Shooter shooter, Turret turret, LedsSubsystem leds) {
        m_drivetrain = drivetrain;
        m_hopper = hopper;
        m_shooter = shooter;
        m_turret = turret;
        m_leds = leds;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        m_frames = MacroManager.getInstance().loadFromFile();
        m_timer.reset();
        m_timer.start();

        // Instantiate matching commands
        m_intakeCommand = new Intake(m_hopper, m_turret, m_leds)
            .withInterruptBehavior(InterruptionBehavior.kCancelSelf)
            .andThen(new ZeroIntake(m_hopper)
                .onlyIf(() -> m_hopper.getHopperState() == HopperState.retracted));

        m_shootCommand = new RPMShootCommand(Constants.ShooterConstants.GND_SPEED_TARGET, m_shooter, m_hopper, m_leds)
            .withInterruptBehavior(InterruptionBehavior.kCancelSelf);

        m_fuelGrabCommand = new FuelGRAB(m_hopper, m_leds)
            .withInterruptBehavior(InterruptionBehavior.kCancelSelf);

        m_agitateCommand = new Agitate(m_hopper, m_leds)
            .withInterruptBehavior(InterruptionBehavior.kCancelSelf);

        m_fuelJammedCommand = new FuelJAMMED(m_hopper, m_shooter)
            .withInterruptBehavior(InterruptionBehavior.kCancelSelf);

        m_zeroTurretCommand = new ZeroTurret(m_turret)
            .withInterruptBehavior(InterruptionBehavior.kCancelSelf);

        m_zeroIntakeCommand = new ZeroIntake(m_hopper)
            .withInterruptBehavior(InterruptionBehavior.kCancelSelf);

        prevB = false;
        prevY = false;
        prevLB = false;
        prevRB = false;
        prevLT = false;
        prevRT = false;
        prevBack = false;

        System.out.println("----- MACRO PLAYBACK STARTED -----");
    }

    @Override
    public void execute() {
        double elapsed = m_timer.get();
        
        // Find the index corresponding to the elapsed time (recording is at 50Hz, so ~20ms steps)
        int index = (int)(elapsed / 0.02);

        if (index >= m_frames.size()) {
            return; // Will end in isFinished
        }

        MacroFrame frame = m_frames.get(index);

        // Drive playback
        m_drivetrain.setControl(m_driveRequest
            .withVelocityX(frame.vx)
            .withVelocityY(frame.vy)
            .withRotationalRate(frame.omega));

        // Button action playback mapping:
        // buttonB -> Intake
        if (frame.buttonB && !prevB) {
            CommandScheduler.getInstance().schedule(m_intakeCommand);
        }

        // buttonY -> ZeroTurret
        if (frame.buttonY && !prevY) {
            CommandScheduler.getInstance().schedule(m_zeroTurretCommand);
        }

        // buttonBack -> ZeroIntake
        if (frame.buttonBack && !prevBack) {
            CommandScheduler.getInstance().schedule(m_zeroIntakeCommand);
        }

        // Left Trigger -> Shoot (whileTrue style)
        if (frame.buttonLT && !prevLT) {
            CommandScheduler.getInstance().schedule(m_shootCommand);
        } else if (!frame.buttonLT && prevLT) {
            m_shootCommand.cancel();
        }

        // Right Trigger -> FuelGrab (whileTrue style)
        if (frame.buttonRT && !prevRT) {
            CommandScheduler.getInstance().schedule(m_fuelGrabCommand);
        } else if (!frame.buttonRT && prevRT) {
            m_fuelGrabCommand.cancel();
        }

        // Right Bumper -> Agitate (whileTrue style)
        if (frame.buttonRB && !prevRB) {
            CommandScheduler.getInstance().schedule(m_agitateCommand);
        } else if (!frame.buttonRB && prevRB) {
            m_agitateCommand.cancel();
        }

        // Left Bumper -> FuelJammed (whileTrue style)
        if (frame.buttonLB && !prevLB) {
            CommandScheduler.getInstance().schedule(m_fuelJammedCommand);
        } else if (!frame.buttonLB && prevLB) {
            m_fuelJammedCommand.cancel();
        }

        // Save current states as previous
        prevB = frame.buttonB;
        prevY = frame.buttonY;
        prevLB = frame.buttonLB;
        prevRB = frame.buttonRB;
        prevLT = frame.buttonLT;
        prevRT = frame.buttonRT;
        prevBack = frame.buttonBack;
    }

    @Override
    public boolean isFinished() {
        return m_timer.get() >= (m_frames.size() * 0.02) || m_frames.isEmpty();
    }

    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
        // Cancel any active playback commands
        if (m_intakeCommand != null) m_intakeCommand.cancel();
        if (m_shootCommand != null) m_shootCommand.cancel();
        if (m_fuelGrabCommand != null) m_fuelGrabCommand.cancel();
        if (m_agitateCommand != null) m_agitateCommand.cancel();
        if (m_fuelJammedCommand != null) m_fuelJammedCommand.cancel();
        if (m_zeroTurretCommand != null) m_zeroTurretCommand.cancel();
        if (m_zeroIntakeCommand != null) m_zeroIntakeCommand.cancel();

        // Stop the drivetrain
        m_drivetrain.setControl(m_driveRequest
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0));

        System.out.println("----- MACRO PLAYBACK FINISHED -----");
    }
}
