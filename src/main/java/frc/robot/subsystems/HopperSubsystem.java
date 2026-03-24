package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.ClimbSubsystem.LockdownMode;

import static edu.wpi.first.units.Units.*;

public class HopperSubsystem extends SubsystemBase {
    private SparkMax leftFuelGrabber;
    private SparkMax hopperFloorMotor;
    private TalonFX hopperExtendMotor;
    // ArmoredCoreAC v4Rusty;

    private DigitalInput hopperExtendSwitch;
    private DigitalInput hopperRetractSwitch;
    // Motor control modes
    // private final NeutralOut m_brake = new NeutralOut();
    private final MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0);

    public enum HopperState {
        retracted,
        extendedUp,
        extendedDown
    };

    public HopperState targetHopperState = HopperState.retracted;

    public Timer m_timer = new Timer();

    /**
    *
    */
    public HopperSubsystem() {

        if (Constants.Debug.INTAKE_ROLLER_EXISTS) {
            leftFuelGrabber = new SparkMax(16, MotorType.kBrushless);

            SparkMaxConfig leftFuelGrabberConfigLeft = new SparkMaxConfig();

            leftFuelGrabberConfigLeft.smartCurrentLimit(10); // Limit gate motor current to 10 A

            leftFuelGrabber.configure(leftFuelGrabberConfigLeft, ResetMode.kResetSafeParameters,
                    PersistMode.kPersistParameters);
        }

        if (Constants.Debug.INTAKE_EXTEND_EXISTS) {

            hopperExtendMotor = new TalonFX(17);
            
            TalonFXConfiguration cfg = new TalonFXConfiguration();

            /* Configure gear ratio */
            FeedbackConfigs fdb = cfg.Feedback;
            fdb.SensorToMechanismRatio = 12.8; // 12.8 rotor rotations per mechanism rotation

            /* Configure Motion Magic */
            MotionMagicConfigs mm = cfg.MotionMagic;
            mm.withMotionMagicCruiseVelocity(RotationsPerSecond.of(1))
                    .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(2)) 
                    // Take approximately 0.1 seconds to reach max accel
                    .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(40));

            Slot0Configs slot0 = cfg.Slot0;
            slot0.kS = 0.25; // Add 0.25 V output to overcome static friction
            slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
            slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
            slot0.kP = 60; // A position error of 0.2 rotations results in 12 V output
            slot0.kI = 0; // No output for integrated error
            slot0.kD = 0.5; // A velocity error of 1 rps results in 0.5 V output


            // Configure current limiting (10 amps)
            CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Constants.IntakeHopperConstants.CURRENT_LIMIT);

            // Set neutral mode to brake
            hopperExtendMotor.setNeutralMode(NeutralModeValue.Coast);

            /* Retry config apply up to 5 times, report if failure */
            StatusCode status = StatusCode.StatusCodeNotInitialized;
            for (int i = 0; i < 5; ++i) {
                status = hopperExtendMotor.getConfigurator().apply(currentLimits);
                if (status.isOK())
                    break;
            }
            if (!status.isOK()) {
                System.out.println("Could not apply configs, error code: " + status.toString());
            }

            if (Constants.Debug.DEBUG_MODE) SmartDashboard.putData("Subsystem: Intake + Hopper + Floor", this);
        }

        hopperFloorMotor = new SparkMax(18, MotorType.kBrushless); // floor motor
        SparkMaxConfig hopperFloorTransferSecureConfig = new SparkMaxConfig();
        hopperFloorTransferSecureConfig.smartCurrentLimit(10); // Limit gate motor current to 10 A
        hopperFloorTransferSecureConfig.inverted(true); // Invert direction of floor transfer motor
        hopperFloorMotor.configure(hopperFloorTransferSecureConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        if (Constants.Debug.INTAKE_EXTEND_EXISTS) {
            hopperExtendSwitch = new DigitalInput(7);
            addChild("hopperExtendSwitch", hopperExtendSwitch);

            hopperRetractSwitch = new DigitalInput(9);
            addChild("hopperRetractSwitch", hopperRetractSwitch);
        }

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if (Constants.Debug.DEBUG_MODE) {
            SmartDashboard.putNumber("Hopper Floor Transfer Speed", hopperFloorMotor.get());
            SmartDashboard.putNumber("Hopper Extend Position", hopperExtendMotor.getPosition().getValueAsDouble());
        }
        if (Constants.Debug.INTAKE_EXTEND_EXISTS) {
            SmartDashboard.putBoolean("Hopper Switch", getHopperRetractSwitch());
        }

        // Check lockdown mode
        if (Robot.getInstance().m_climb.currentLockdownMode == LockdownMode.engaged) {
            // In lockdown mode, ensure hopper is retracted
            // setHopperRetract();
            if (targetHopperState == HopperState.retracted) {
                setHopperRetract();
            }
        }

        hopperExtendMotorControl();
    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void setFuelGrabberSpeed() {
        if (Constants.Debug.INTAKE_ROLLER_EXISTS) {
            leftFuelGrabber.set(1); // 1
        }
    }

    public void stopFuelGrabber() {
        if (Constants.Debug.INTAKE_ROLLER_EXISTS) {
            leftFuelGrabber.set(0);
        }
    }

    public void setHopperFloorTransferSecureSpeed(double speed) {
        hopperFloorMotor.set(speed);
    }

    public void stopHopperFloorTransferSecure() {
        hopperFloorMotor.set(0);
    }

    public void ifIntakeJammed() {
        if (Constants.Debug.INTAKE_ROLLER_EXISTS) {
            leftFuelGrabber.set(-0.2);
        }
    }

    public void setHopperExtendDown() {
        if (Robot.getInstance().m_climb.currentLockdownMode == LockdownMode.engaged) {
            // In lockdown mode, don't extend
            return;
        } else {
            targetHopperState = HopperState.extendedDown;

            if (Constants.Debug.DEBUG_MODE)
                System.out.println("Extending hopper");
        }
    }

    public void setHopperExtendUp() {
        if (Robot.getInstance().m_climb.currentLockdownMode == LockdownMode.engaged) {
            // In lockdown mode, don't extend
            return;
        } else {
            targetHopperState = HopperState.extendedUp;

            if (Constants.Debug.DEBUG_MODE)
                System.out.println("Extending hopper");
        }
    }

    public void setHopperRetract() {
        targetHopperState = HopperState.retracted;

        if (Constants.Debug.DEBUG_MODE)
            System.out.println("Retracting hopper");
    }

    public boolean isHopperExtendedUp() {
        if (Constants.Debug.INTAKE_EXTEND_EXISTS) {
            return Math.abs(hopperExtendMotor.getPosition().getValueAsDouble() - Constants.IntakeHopperConstants.EXTENDED_UP_POSITION_POSITION) <= Constants.IntakeHopperConstants.MOTOR_TOLERANCE;
        } else {
            return true; // Assume not extended when intake doesn't exist
            // TODO false???
        }
    }
    public boolean isHopperExtendedDown() {
        if (Constants.Debug.INTAKE_EXTEND_EXISTS) {
            return Math.abs(hopperExtendMotor.getPosition().getValueAsDouble() - Constants.IntakeHopperConstants.EXTENDED_DOWN_POSITION_POSITION) <= Constants.IntakeHopperConstants.MOTOR_TOLERANCE;
        } else {
            return true; // Assume not extended when intake doesn't exist
            // TODO false???
        }
    }

    public boolean isHopperRetracted() {
        if (Constants.Debug.INTAKE_EXTEND_EXISTS) {
            // return hopperRetractSwitch.get();
            return Math.abs(hopperExtendMotor.getPosition().getValueAsDouble() - Constants.IntakeHopperConstants.RETRACT_POSITION_POSITION) < Constants.IntakeHopperConstants.MOTOR_TOLERANCE;
        } else {
            return true; // Assume retracted when intake doesn't exist
        }
    }

    public boolean getHopperRetractSwitch() {
        if (Constants.Debug.INTAKE_EXTEND_EXISTS) {
            return hopperRetractSwitch.get();
        } else {
            return true; // Assume retracted when intake doesn't exist
        }
    }

    public boolean isHopperAtPosition() {
        switch (targetHopperState) {
            case extendedUp:
                return isHopperExtendedUp();
            case extendedDown:
                return isHopperExtendedDown();
            case retracted:
                return isHopperRetracted();
            default:
                return true;
        }
    }

    public HopperState getHopperState() {
        return targetHopperState;
    }

    public void hopperExtendMotorControl() {
        if (Constants.Debug.INTAKE_EXTEND_EXISTS) {
            if (targetHopperState == HopperState.retracted) {
                hopperExtendMotor.setControl(m_mmReq.withPosition(Constants.IntakeHopperConstants.RETRACT_POSITION_POSITION));
            }
            else if (targetHopperState == HopperState.extendedUp) {
                hopperExtendMotor.setControl(m_mmReq.withPosition(Constants.IntakeHopperConstants.EXTENDED_UP_POSITION_POSITION));
            }
            else {
                hopperExtendMotor.setControl(m_mmReq.withPosition(Constants.IntakeHopperConstants.EXTENDED_DOWN_POSITION_POSITION));
            }
            // if (isExtending) {
            //     // if (!isHopperExtended()) {
            //     // hopperExtendMotor.set(Constants.IntakeHopperConstants.EXTEND_SPEED); //
            //     // Extend at half speed
            //     // } else {
            //     // hopperExtendMotor.set(0); // Stop when fully extended
            //     // }

            //     if (!m_timer.hasElapsed(Constants.IntakeHopperConstants.EXTEND_TIME_SECONDS)) {
            //         hopperExtendMotor.set(Constants.IntakeHopperConstants.EXTEND_SPEED);
            //     } else {
            //         hopperExtendMotor.set(Constants.IntakeHopperConstants.HOLD_SPEED);
            //     }

            //     if (!m_timer.isRunning()) {
            //         m_timer.restart();
            //     }

            // } else {
            //     if (!isHopperRetracted()) {
            //         hopperExtendMotor.set(Constants.IntakeHopperConstants.RETRACT_SPEED); // Retract at half speed
            //     } else {
            //         hopperExtendMotor.set(0); // Stop when fully retracted
            //     }

            //     if (m_timer.isRunning()) {
            //         m_timer.stop();
            //     }

            // }
        }
    }

    public void forwardForAgitate() {
        hopperFloorMotor.set(Constants.IntakeHopperConstants.AGITATE_COMMAND_SPEED);
    }

    public void reverseForAgitate() {
        hopperFloorMotor.set(-Constants.IntakeHopperConstants.AGITATE_COMMAND_SPEED);
    }

    public void stopAll() {
        stopFuelGrabber();
        stopHopperFloorTransferSecure();

    }

    public void stopHopperExtend() {
        hopperExtendMotor.set(0);
    }

    public void hopperZeroing() {
        hopperExtendMotor.set(Constants.IntakeHopperConstants.HOPPER_ZEROING_SPEED);
    }

    public void resetPosition() {
        hopperExtendMotor.setPosition(0);
    }
}