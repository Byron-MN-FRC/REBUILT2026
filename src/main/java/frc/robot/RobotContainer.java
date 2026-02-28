// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.Agitate;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AutonExtend;
import frc.robot.commands.AutonRetract;
import frc.robot.commands.AutonShootCommand;
import frc.robot.commands.AutonStart;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.ClimbLowerAuto;
import frc.robot.commands.ClimbRaiseAuto;
import frc.robot.commands.ClimbZeroing;
import frc.robot.commands.FloorTransfer;
import frc.robot.commands.FuelGRAB;
import frc.robot.commands.FuelJAMMED;
import frc.robot.commands.Intake;
import frc.robot.commands.Agitate;
import frc.robot.commands.Lock45Degrees;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.TrackHub;
import frc.robot.commands.ledtestcommands.fasterfaster;
import frc.robot.commands.ledtestcommands.flash;
// import frc.robot.commands.Retract;
// import frc.robot.commands.Extend;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.LedsSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class RobotContainer {

    public final Shooter m_shooter = new Shooter();
    public final Turret m_turret = new Turret();
    public final ClimbSubsystem m_climb = new ClimbSubsystem();
    public final Hopper m_hopper = new Hopper();
    public final LedsSubsystem m_leds = new LedsSubsystem();

    private final DigitalOutput pointer = new DigitalOutput(3);
    SendableChooser<Command> m_chooser = new SendableChooser<>();

    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                        // speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final CommandXboxController gamepad = new CommandXboxController(0);
    private final CommandXboxController accessory = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    // public final ColorLED lightStrip = new ColorLED(LED_PORT, LED_LENGTHS);
    public final Field2d m_field = new Field2d();
    public final Field2d m_autoField = new Field2d();

    public RobotContainer() {
        // pointer.set(true);
        NamedCommands.registerCommand("AutonStart", new AutonStart(m_turret, m_climb, m_leds));
        NamedCommands.registerCommand("AutonRetract", new AutonRetract(m_hopper));
        NamedCommands.registerCommand("AutonExtend", new AutonExtend(m_hopper, m_leds));
        NamedCommands.registerCommand("AutonShootCommand", new ShootCommand(m_shooter, m_hopper, m_leds));
        NamedCommands.registerCommand("ClimbRaiseAuto", new ClimbRaiseAuto(m_climb, m_leds));
        NamedCommands.registerCommand("ClimbLowerAuto", new ClimbLowerAuto(m_climb, m_leds));

        if (Constants.Debug.DEBUG_MODE) {
            SmartDashboard.putData("ClimbRaiseAuto", new ClimbRaiseAuto(m_climb, m_leds));
            SmartDashboard.putData("ClimbLowerAuto", new ClimbLowerAuto(m_climb, m_leds));
            SmartDashboard.putData("Agitate", new Agitate(m_hopper, m_leds));
        }
        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=SMARTDASHBOARD
        SmartDashboard.putData("Robot Position", m_field);
        SmartDashboard.putData("Auto Field", m_autoField);

        if (Constants.Debug.DEBUG_MODE) {
            SmartDashboard.putData("Climb", new ClimbCommand(m_climb, m_leds, m_hopper, m_turret));
            SmartDashboard.putData("Zeroing", new ClimbZeroing(m_climb, m_leds));

            SmartDashboard.putData("FuelGRAB", new FuelGRAB(m_hopper, m_leds));
            // SmartDashboard.putData("Extend", new Extend(m_hopper));
            // SmartDashboard.putData("Retract", new Retract(m_hopper));

            SmartDashboard.putData("Intake", new Intake(m_hopper,m_turret, m_leds));
            
            SmartDashboard.putData("FasterFasterLights", new fasterfaster(m_leds));
            SmartDashboard.putData("Flashing lights", new flash(m_leds));
        }

        // Configure the button bindings
        configureBindings();

        m_chooser = AutoBuilder.buildAutoChooser();
        m_chooser.onChange(new Consumer<Command>() {
            public void accept(Command t) {
                // m_vision.updateAutoStartPosition(m_chooser.getSelected().getName());
                Command selectedCommand = m_chooser.getSelected();
                if (selectedCommand != null) {
                    String autoName = selectedCommand.getName();
                    if (autoName != null && !autoName.isEmpty()) {
                        List<Pose2d> poses = getPathPoses(autoName);
                        if (poses != null && !poses.isEmpty()) {
                            setFieldTrajectory(poses, m_autoField);
                        } else {
                            poses = new ArrayList<>();
                            poses.add(new Pose2d(0, 0, new Rotation2d(0)));
                            setFieldTrajectory(poses, m_autoField);
                        }
                    }
                }
            }
        });
        SmartDashboard.putData("Auto Mode", m_chooser);

        PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            m_field.setRobotPose(pose);
        });

        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            m_field.getObject("target pose").setPose(pose);
        });

        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            m_field.getObject("path").setPoses(poses);
        });

    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() ->

                drive.withVelocityX(-gamepad.getLeftY() * MaxSpeed * m_climb.lockdownDriveControl) // Drive forward with
                                                                                                   // negative Y
                                                                                                   // (forward)
                        .withVelocityY(-gamepad.getLeftX() * MaxSpeed * m_climb.lockdownDriveControl) // Drive left with
                                                                                                      // negative X
                                                                                                      // (left)
                        .withRotationalRate(-gamepad.getRightX() * MaxAngularRate * m_climb.lockdownDriveControl) // Drive
                                                                                                                  // counterclockwise
                                                                                                                  // with
                                                                                                                  // negative
                                                                                                                  // X
                                                                                                                  // (left)
                ));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        // gamepad.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // gamepad.b().whileTrue(drivetrain.applyRequest(() ->
        // point.withModuleDirection(new Rotation2d(-gamepad.getLeftY(),
        // -gamepad.getLeftX()))
        // ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // gamepad.back().and(gamepad.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // gamepad.back().and(gamepad.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // gamepad.start().and(gamepad.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // gamepad.start().and(gamepad.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        // gamepad.start().and(gamepad.back()).onTrue(new InstantCommand(() ->
        // SignalLogger.stop()).andThen(new InstantCommand(()
        // ->System.out.println("Stopping Loger"))));

        // Reset the field-centric heading on left bumper press.
        gamepad.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        gamepad.leftBumper()
                .whileTrue(new Lock45Degrees(drivetrain).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

        drivetrain.registerTelemetry(logger::telemeterize);

        accessory.y().onTrue(new ClimbCommand(m_climb, m_leds, m_hopper, m_turret)
                .withInterruptBehavior(InterruptionBehavior.kCancelSelf));

        accessory.b().onTrue(new ClimbZeroing(m_climb, m_leds).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

        accessory.a().toggleOnTrue(new FloorTransfer(m_hopper).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

        accessory.start().onTrue(m_turret.checkZeroLeft().withInterruptBehavior(InterruptionBehavior.kCancelSelf));

        accessory.back().onTrue(new InstantCommand(() -> m_turret.resetPosition())
                .withInterruptBehavior(InterruptionBehavior.kCancelSelf));

        // accessory.rightTrigger().whileTrue(new ShooterSpin( m_turret, m_leds
        // ).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

        accessory.leftBumper()
                .toggleOnTrue(new TrackHub(m_turret, m_leds).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

        accessory.rightTrigger()
                .whileTrue(new ShootCommand(m_shooter,m_hopper,m_leds).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

        gamepad.rightTrigger()
                .whileTrue(new FuelGRAB(m_hopper, m_leds).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

        gamepad.b().onTrue(new Intake(m_hopper, m_turret, m_leds).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    
        gamepad.rightBumper().onTrue(new FuelJAMMED(m_hopper, m_shooter).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

        gamepad.leftTrigger().whileTrue(new Agitate(m_hopper, m_leds).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    }

    public CommandXboxController getaccessory() {
        return accessory;
    }

    public Command getAutonomousCommand() {
        return m_chooser.getSelected();
    }

    public List<Pose2d> getPathPoses(String autoName) {
        List<PathPlannerPath> paths;
        try {
            paths = PathPlannerAuto.getPathGroupFromAutoFile(autoName);
        } catch (Exception e) {
            System.out.println(e.getMessage());
            paths = new ArrayList<>();
        }
        List<Pose2d> poses = new ArrayList<>();
        for (PathPlannerPath path : paths) {
            poses.addAll(path.getPathPoses());
        }
        return poses;
    }

    public void setFieldTrajectory(List<Pose2d> poses, Field2d field) {
        field.getObject("trajectory").setPoses(poses);
        field.setRobotPose(poses.get(0));

    }
}
