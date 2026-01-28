package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.Optional;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 *
 */
public class DriveToPosition extends Command {

    private static final TrapezoidProfile.Constraints Magnitude_Constraints = new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(8, 8);
    
    private final ProfiledPIDController magnitudeController = new ProfiledPIDController(2.75, 0, 0, Magnitude_Constraints);
    private final ProfiledPIDController omegaController = new ProfiledPIDController(4, 0, .1, OMEGA_CONSTRAINTS);

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity    
    
    private final CommandSwerveDrivetrain drivetrain;
    private Pose2d goalPose;
    private Pose2d currentPose;
    
    /**
     * @param subsystem
     * @param goalPose
     */
    public DriveToPosition(CommandSwerveDrivetrain subsystem, Pose2d goalPose) {
        drivetrain = subsystem;
        this.goalPose = goalPose;

        omegaController.setTolerance(Units.degreesToRadians(1.5));
        omegaController.enableContinuousInput(-Math.PI, Math.PI);
        magnitudeController.setTolerance(0.04); // 4 cm tolerance

        addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        currentPose = drivetrain.getState().Pose;
        magnitudeController.reset(currentPose.getTranslation().getDistance(goalPose.getTranslation()));
        omegaController.reset(currentPose.getRotation().getRadians());
        
        magnitudeController.setGoal(0);
        omegaController.setGoal(goalPose.getRotation().getRadians());

    // Optional: set the dashboard field target for visualization.
    // Robot.getInstance().targetPoseField.setRobotPose(goalPose);
    }
    
    // // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        
        // Calculate distances and angles
        currentPose = drivetrain.getState().Pose;
        double currentDistance = currentPose.getTranslation().getDistance(goalPose.getTranslation());
        double yDistance = goalPose.getTranslation().getY() - currentPose.getTranslation().getY();
        double xDistance = goalPose.getTranslation().getX() - currentPose.getTranslation().getX();
        double angle = Math.atan2(yDistance, xDistance);
        
        // Calculate speeds from controllers
        var combinedSpeed = magnitudeController.calculate(currentDistance);
        
        double xSpeedFromPolar = Math.cos(angle) * combinedSpeed;
        double ySpeedFromPolar = Math.sin(angle) * combinedSpeed;

        if (magnitudeController.atGoal()) {
            xSpeedFromPolar = 0;
            ySpeedFromPolar = 0;
        }

        var omegaSpeed = omegaController.calculate(currentPose.getRotation().getRadians());
        if (omegaController.atGoal()) {
            omegaSpeed = 0;
        }

        /*
        * DEBUGGING INFO
        * 
        * SmartDashboard.putString("goal pose", goalPose.toString());
        * SmartDashboard.putString("currentPose", drivetrain.getState().Pose.toString());
        * SmartDashboard.putNumber("combinedSpeed", combinedSpeed);
        * SmartDashboard.putNumber("xSpeedFromPolar", xSpeedFromPolar);
        * SmartDashboard.putNumber("ySpeedFromPolar", ySpeedFromPolar);
        * SmartDashboard.putNumber("angleRJIEOFOS", angle * (180/Math.PI));
        * SmartDashboard.putNumber("cR", currentDistance);
        * SmartDashboard.putNumber("dCXGX", xDistance);
        *
        */
        
        // Drive

        Optional<Alliance> ally = DriverStation.getAlliance();
        
        if (ally.get() == Alliance.Red) {
            drivetrain.setControl(
            Robot.getInstance().drive
            .withVelocityX(xSpeedFromPolar * MaxSpeed)
            .withVelocityY(ySpeedFromPolar * MaxSpeed)
            .withRotationalRate(omegaSpeed * MaxAngularRate)
            );
        } else {
            drivetrain.setControl(
                Robot.getInstance().drive
                .withVelocityX(-xSpeedFromPolar * MaxSpeed)
            .withVelocityY(-ySpeedFromPolar * MaxSpeed)
            .withRotationalRate(omegaSpeed * MaxAngularRate)
            );
        }
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return magnitudeController.atGoal() && omegaController.atGoal();
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}
