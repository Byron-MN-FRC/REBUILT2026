// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Lock45Degrees extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private Rotation2d targetAngle;
  private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
  
  private final SwerveRequest.FieldCentricFacingAngle drive45 = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage) // Use open-loop control for drive motors
            .withHeadingPID(7.5, 0, 0);

    /** Creates a new Lock45Degrees. */
  public Lock45Degrees(CommandSwerveDrivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      double currentAngle = drivetrain.getState().Pose.getRotation().getDegrees();
      if (currentAngle >= 0 && currentAngle < 90) {
          targetAngle = new Rotation2d(Math.PI / 4); // 45 degrees
      } else if (currentAngle >= 90 && currentAngle < 180) {
          targetAngle = new Rotation2d(3 * Math.PI / 4); // 135 degrees
      } else if (currentAngle >= -180 && currentAngle < -90) {
          targetAngle = new Rotation2d(-3 * Math.PI / 4); // -135 degrees
      } else {
          targetAngle = new Rotation2d(-Math.PI / 4); // -45 degrees
      }


      if (Constants.Debug.DEBUG_MODE) {
        SmartDashboard.putNumber("current a", currentAngle);
        SmartDashboard.putNumber("target a", targetAngle.getDegrees());
      }

      if (Constants.DriveConstants.MyAlliance() == DriverStation.Alliance.Red) {
          targetAngle = targetAngle.plus(new Rotation2d(Math.PI)); // Flip the angle for the red alliance
      }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.setControl(
                drive45.withTargetDirection(targetAngle) // Lock the robot at 45 degrees
                    .withVelocityX(-Robot.getInstance().gamepad.getLeftY() * MaxSpeed * 0.75) // Drive forward with negative Y (forward)
                    .withVelocityY(-Robot.getInstance().gamepad.getLeftX() * MaxSpeed * 0.75) // Drive left with negative X (left)
            );

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
