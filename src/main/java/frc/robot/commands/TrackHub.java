// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.RobotCentric;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.TurretCam;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.leds;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TrackHub extends Command {

  private Turret m_turret;
  private final leds m_leds;

  /** Creates a new trackHub. */
  public TrackHub(Turret subsystem, leds leds) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    addRequirements(leds);
    m_leds = leds;
    m_turret = subsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_turret.aim((m_turret.rotateShooterMotor.getPosition().getValueAsDouble() * 360) + TurretCam.getAngleError());
    if (TurretCam.getAngleError() == 0 && TurretCam.targetLocated() == true) {
      m_leds.setColorGreen();
    }
    else if(TurretCam.getAngleError() <= 5 && TurretCam.targetLocated() == false) {
      m_leds.setColorYellow();
    }
    else{
      m_leds.setColorRed();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_turret.spinStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}