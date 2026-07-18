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

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// POVButton removed — outreach mode uses a single gamepad only
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.Agitate;
import frc.robot.commands.AutonExtend;
import frc.robot.commands.AutonIntake;
import frc.robot.commands.AutonIntakestop;
import frc.robot.commands.AutonRetract;
import frc.robot.commands.AutonSetAiming;
import frc.robot.commands.AutonStart;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.ClimbLowerAuto;
import frc.robot.commands.ClimbRaiseAuto;
import frc.robot.commands.ClimbZeroing;
import frc.robot.commands.FloorTransfer;
import frc.robot.commands.FuelGRAB;
import frc.robot.commands.FuelJAMMED;
import frc.robot.commands.Intake;
import frc.robot.commands.Lock45Degrees;
import frc.robot.commands.RPMShootCommand;
import frc.robot.commands.SnowBlowerCommandGroup;
import frc.robot.commands.TrackHub;
import frc.robot.commands.TrackHubNew;
import frc.robot.commands.ZeroIntake;
import frc.robot.commands.ZeroTurret;
import frc.robot.commands.ledtestcommands.fasterfaster;
import frc.robot.commands.ledtestcommands.flash;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.HopperSubsystem.HopperState;
import frc.robot.subsystems.LedsSubsystem;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Vision;

public class RobotContainer {

    public final Shooter m_shooter = new Shooter();
    public final Turret m_turret = new Turret();
    public final ClimbSubsystem m_climb = new ClimbSubsystem();
    public final HopperSubsystem m_hopper = new HopperSubsystem();
    public final LedsSubsystem m_leds = new LedsSubsystem();

   
    SendableChooser<Command> m_chooser = new SendableChooser<>();

    // Speed constants for slow (outreach) and full speed modes
    private static final double SLOW_SPEED = 0.4 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private static final double SLOW_ANGULAR_RATE = RotationsPerSecond.of(0.75 * 0.70).in(RadiansPerSecond);
    private static final double FULL_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private static final double FULL_ANGULAR_RATE = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    // Active speed — starts in slow (outreach) mode
    private double MaxSpeed = SLOW_SPEED;
    private double MaxAngularRate = SLOW_ANGULAR_RATE;
    private boolean fullSpeedMode = false;

    /* Setting up bindings for necessary control of the swerve drive platform */
    public final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    // Outreach mode: single gamepad only (port 0) — no accessory controller
    public final CommandXboxController gamepad = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Vision m_vision = new Vision();

    // TODO fix
    public TagApproaches tagApproaches = new TagApproaches();
    public final Field2d m_field = new Field2d();
    public final Field2d m_autoField = new Field2d();

    public RobotContainer() {
        NamedCommands.registerCommand("AutonStart", new AutonStart(m_turret, m_climb, m_leds, m_hopper));
        NamedCommands.registerCommand("AutonRetract", new AutonRetract(m_hopper));
        NamedCommands.registerCommand("AutonExtend", new AutonExtend(m_hopper, m_leds));
        NamedCommands.registerCommand("AutonIntake", new AutonIntake(m_hopper));
        NamedCommands.registerCommand("AutonIntakestop", new AutonIntakestop(m_hopper));
        
        NamedCommands.registerCommand("OutpostShootRPM", new RPMShootCommand(Constants.ShooterConstants.HIGH_SPEED_TARGET, m_shooter, m_hopper, m_leds));
        NamedCommands.registerCommand("DepotShootRPM", new RPMShootCommand(Constants.ShooterConstants.HIGH_SPEED_TARGET, m_shooter, m_hopper, m_leds));
        NamedCommands.registerCommand("ClimbShootRPM", new RPMShootCommand(Constants.ShooterConstants.LOW_SPEED_TARGET, m_shooter, m_hopper, m_leds));
        NamedCommands.registerCommand("LeftClimbShootRPM", new RPMShootCommand(Constants.ShooterConstants.LOW_SPEED_TARGET -100, m_shooter, m_hopper, m_leds));
        NamedCommands.registerCommand("LeftShootOnlyRPM", new RPMShootCommand(Constants.ShooterConstants.HIGH_SPEED_TARGET -50, m_shooter, m_hopper, m_leds));
        NamedCommands.registerCommand("MidShootRPM", new RPMShootCommand(2350, m_shooter, m_hopper, m_leds));

        NamedCommands.registerCommand("OutpostAim", new AutonSetAiming(m_turret, -42.5));

         NamedCommands.registerCommand("DepotAim", new AutonSetAiming(m_turret, 0));
         NamedCommands.registerCommand("TrackHubNew", new TrackHubNew(m_turret, m_leds));


        NamedCommands.registerCommand("ClimbRaiseAuto", new ClimbRaiseAuto(m_climb, m_leds));
        NamedCommands.registerCommand("ClimbLowerAuto", new ClimbLowerAuto(m_climb, m_leds));
        NamedCommands.registerCommand("TrackHub", new TrackHub(m_turret, m_leds));
        SmartDashboard.putData("Robot Position", m_field);
        SmartDashboard.putData("Auto Field", m_autoField);

        if (Constants.Debug.DEBUG_MODE) {
            SmartDashboard.putData("ClimbRaiseAuto", new ClimbRaiseAuto(m_climb, m_leds));
            SmartDashboard.putData("ClimbLowerAuto", new ClimbLowerAuto(m_climb, m_leds));
            SmartDashboard.putData("Agitate", new Agitate(m_hopper, m_leds));
            SmartDashboard.putData("Climb", new ClimbCommand(m_climb, m_leds, m_hopper, m_turret));
            SmartDashboard.putData("Zeroing", new ClimbZeroing(m_climb, m_leds));
            SmartDashboard.putData("FuelGRAB", new FuelGRAB(m_hopper, m_leds));
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
        // ---------------------------------------------------------------
        // OUTREACH MODE — single CommandXboxController (port 0)
        // Speed: 40% translation, 70% of nominal angular rate
        // Turret: fixed at default (no auto-tracking)
        // Climb: removed for outreach safety
        // ---------------------------------------------------------------

        // Left Stick = translation, Right Stick X = rotation (field-centric)
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() ->
                        drive.withVelocityX(-gamepad.getLeftY() * MaxSpeed)   // forward/back
                             .withVelocityY(-gamepad.getLeftX() * MaxSpeed)   // strafe
                             .withRotationalRate(-gamepad.getRightX() * MaxAngularRate) // turn
                ));

        // Idle while disabled — keeps neutral mode applied to drive motors
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        drivetrain.registerTelemetry(logger::telemeterize);

        // START — reset field-centric heading
        gamepad.start().onTrue(
                new InstantCommand(() -> m_vision.tempDisable(0.5))
                        .andThen(drivetrain.runOnce(() -> drivetrain.seedFieldCentric())));

        // B — Intake (pick up game piece; auto-zeros intake on retract)
        gamepad.b().onTrue(
                new Intake(m_hopper, m_turret, m_leds)
                        .withInterruptBehavior(InterruptionBehavior.kCancelSelf)
                        .andThen(new ZeroIntake(m_hopper)
                                .onlyIf(() -> m_hopper.getHopperState() == HopperState.retracted)));

        // A — Floor Transfer (toggle: press once to start, press again to stop)
        // gamepad.a().toggleOnTrue(
        //         new FloorTransfer(m_hopper)
        //                 .withInterruptBehavior(InterruptionBehavior.kCancelSelf));

        // Right Trigger — FuelGRAB (hold to grab/feed fuel into hopper)
        gamepad.rightTrigger().whileTrue(
                new FuelGRAB(m_hopper, m_leds)
                        .withInterruptBehavior(InterruptionBehavior.kCancelSelf));

        // Left Trigger — Shoot at GND_SPEED (hold to spin up and fire)
        gamepad.leftTrigger().whileTrue(
                new RPMShootCommand(Constants.ShooterConstants.GND_SPEED_TARGET, m_shooter, m_hopper, m_leds)
                        .withInterruptBehavior(InterruptionBehavior.kCancelSelf));

        // Right Bumper — Agitate (hold to unjam hopper)
        gamepad.rightBumper().whileTrue(
                new Agitate(m_hopper, m_leds)
                        .withInterruptBehavior(InterruptionBehavior.kCancelSelf));

        // Back — ZeroIntake (manual intake reset)
        gamepad.back().onTrue(
                new ZeroIntake(m_hopper)
                        .withInterruptBehavior(InterruptionBehavior.kCancelSelf));
        
        gamepad.leftBumper().whileTrue(
                new FuelJAMMED(m_hopper, m_shooter)
                        .withInterruptBehavior(InterruptionBehavior.kCancelSelf));

        gamepad.y().onTrue(
                new ZeroTurret(m_turret)
                        .withInterruptBehavior(InterruptionBehavior.kCancelSelf));

        // X — Toggle full speed mode vs slow (outreach) mode
        gamepad.x().onTrue(new InstantCommand(() -> toggleSpeedMode()));

    }

    /** Toggle between slow (outreach) and full speed driving modes. */
    private void toggleSpeedMode() {
        fullSpeedMode = !fullSpeedMode;
        if (fullSpeedMode) {
            MaxSpeed = FULL_SPEED;
            MaxAngularRate = FULL_ANGULAR_RATE;
        } else {
            MaxSpeed = SLOW_SPEED;
            MaxAngularRate = SLOW_ANGULAR_RATE;
        }
    }

    // getaccessory() removed — outreach mode uses a single gamepad only

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
