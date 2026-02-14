// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class StopCommands extends Command {
  Hopper hopper;
  Shooter shooter;
  ClimbSubsystem climbSubsystem;
  Turret turret;

  /** Creates a new StopCommands. */
  public StopCommands(Hopper hopper, Shooter shooter, ClimbSubsystem climbSubsystem, Turret turret) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(hopper);
    addRequirements(shooter);
    addRequirements(climbSubsystem);
    addRequirements(turret);
    this.hopper = hopper;
    this.shooter = shooter;
    this.climbSubsystem = climbSubsystem;
    this.turret = turret;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hopper.stopHopperFloorTransferSecure();
    hopper.stopFuelGrabber();
    shooter.stopAll();
    climbSubsystem.stopClimbing();
    turret.spinStop();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
