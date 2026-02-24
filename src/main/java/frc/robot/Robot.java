// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    

    public AddressableLED m_led;
    private Command m_autonomousCommand;
    // private AddressableLEDBuffer m_ledBuffer;
    //private int m_rainbowFirstPixelHue;
    // private Color[] redWhiteArray = {Color.kBlueViolet, Color.kBlue};
    // private Color[] blueWhiteArray = {Color.kBlue, Color.kWhite};

    private static final RobotContainer m_robotContainer = new RobotContainer();
    //public static final ColorLED ColorLED = new ColorLED();
    //private RobotContainer m_robotContainer;
    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        // This is literally the government tracker
        HAL.report(tResourceType.kResourceType_Framework, tInstances.kFramework_RobotBuilder);
        enableLiveWindowInTest(true);
    }
    
    public static RobotContainer getInstance(){
        return m_robotContainer;
    }
    
    @Override
    public void robotInit() {}
        
    @Override
    public void robotPeriodic() {
            m_timeAndJoystickReplay.update();
            CommandScheduler.getInstance().run(); 
            m_robotContainer.m_field.setRobotPose(m_robotContainer.drivetrain.getState().Pose);
            
            SmartDashboard.putNumber("Robot Gyro", m_robotContainer.drivetrain.getState().Pose.getRotation().getDegrees());

            if (Constants.Debug.DEBUG_MODE) SmartDashboard.putNumber("Turret Distance to Hub", TurretCam.getDistance());
            if (Constants.Debug.DEBUG_MODE) SmartDashboard.putNumber("Rotational Speed", m_robotContainer.drivetrain.getState().Speeds.omegaRadiansPerSecond);
    }
        
    @Override
    public void disabledInit() {
        Robot.getInstance().m_climb.stopAll();
        Robot.getInstance().m_hopper.stopAll();
        Robot.getInstance().m_turret.stopAll();
        Robot.getInstance().m_shooter.stopAll();
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}
