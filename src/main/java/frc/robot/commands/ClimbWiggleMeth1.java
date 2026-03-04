// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimbWiggleMeth1 extends Command {
    private CommandSwerveDrivetrain drivetrain;
    private Timer timer;
    private double switchTime = 0.2; // seconds
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(Units.MetersPerSecond); // kSpeedAt12Volts desired
     private double vel = 0.25 * MaxSpeed; // 25% of max speed

                                                                                              // top
    // speed
    private double MaxAngularRate = Units.RotationsPerSecond.of(0.75).in(Units.RadiansPerSecond); // 3/4 of a rotation
                                                                                                  // per second
    // max angular velocity

    private final SwerveRequest.FieldCentric wigglyrequest = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    /** Creates a new ClimbWiggleMeth1. */
    public ClimbWiggleMeth1(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
        // initialize the timer instance
        timer = new Timer();
        if (Constants.Debug.DEBUG_MODE) {
            SmartDashboard.putNumber("ClimbWiggleSwitchTime", switchTime);
        }
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        switchTime = SmartDashboard.getNumber("ClimbWiggleSwitchTime", switchTime); // Update switchTime from SmartDashboard

        if (timer.hasElapsed(switchTime)) { // Check if switchTime seconds have passed
            vel *= -1; // Reverse direction
            timer.reset(); // Reset the timer after executing the wiggle command
            timer.start();
        }

        drivetrain.setControl(wigglyrequest
                .withVelocityX(vel) // Move forward/back at 25% of max speed
                .withVelocityY(0.0)
                .withRotationalRate(0));
    }

    @Override
    public void end(boolean interrupted) {
        // stop motion when command ends
        drivetrain.setControl(wigglyrequest.withVelocityX(0.0).withVelocityY(0.0).withRotationalRate(0.0));
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return false; // run until interrupted
    }

}
