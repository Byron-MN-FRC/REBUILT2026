// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.TurretCam;
import frc.robot.subsystems.LedsSubsystem;
import frc.robot.subsystems.Turret;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TrackHub extends Command {

  private Turret m_turret;
  private final LedsSubsystem m_leds;
  private Timer m_timer = new Timer();
  public boolean isActive;

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
    isActive = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (TurretCam.targetLocated() == true) {
      m_turret
          .aimRelativeDegrees((m_turret.getAngleDegrees()) + TurretCam.getAngleError());
      m_timer.stop();
      m_timer.reset();

    } else if (!TurretCam.targetLocated() && !m_timer.isRunning()) {
      m_timer.start();
    }

    if (m_timer.hasElapsed(Constants.TurretConstants.TURRET_CAM_TIMEOUT)) {
      m_turret.aimDegrees(Constants.TurretConstants.NEUTRAL_POSITION);
    }


    // Generate LED colors based on how close the turret angle is to the target.
    // Green is on target, yellow is close, and red is far.
    if (TurretCam.getAngleError() == 0 && TurretCam.targetLocated() == true) {
      if (m_leds.usingSubsystem == LedsSubsystem.SubsystemUsingLEDS.turret) {
        m_leds.setColorGreen();
      }
    } else if (TurretCam.getAngleError() <= 5 && TurretCam.targetLocated() == false) {
      if (m_leds.usingSubsystem == LedsSubsystem.SubsystemUsingLEDS.turret) {
        m_leds.setColorYellow();
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
    m_turret.spinStop();
    m_leds.noSubsystemUsingLeds();
    isActive = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}