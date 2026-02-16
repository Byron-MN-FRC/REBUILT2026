// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.ctre.phoenix6.HootAutoReplay;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    

    public AddressableLED m_led;
    private Command m_autonomousCommand;
    private AddressableLEDBuffer m_ledBuffer;
    //private int m_rainbowFirstPixelHue;
    private Color[] redWhiteArray = {Color.kBlueViolet, Color.kBlue};
    private Color[] blueWhiteArray = {Color.kBlue, Color.kWhite};

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
        SmartDashboard.putNumber("Robot Gyro", m_robotContainer.drivetrain.getState().Pose.getRotation().getDegrees());
    }

    public static RobotContainer getInstance(){
        return m_robotContainer;
    }

    @Override
    public void robotInit() {
       // GenericEntry myEntry  = mytab.add("LEDs", 1).getEntry();
       // Double red = myEntry.getDouble(0);
        //PWM port 9

        //Must be a PWM header, not MXP or DIO

        //PORT
        //m_led = new AddressableLED(1);

        // Reuse buffer

        //Default to a length of 60, start empty output

        //Length is expensive to set, so only set it once, then just update data

        //m_ledBuffer = new AddressableLEDBuffer(24);
//144 lights on big strand
//75 seems to be max amount before something explodes
//8 lights on small strand
        //m_led.setLength(m_ledBuffer.getLength());

        // Set the data

        //m_led.setData(m_ledBuffer);

        //m_led.start();

        //for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        //    // Sets the specified LED to the HSV values for red
        //    m_ledBuffer.setHSV(i, 0, 100, 100);
        //}

        //m_led.setData(m_ledBuffer);
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run(); 
        m_robotContainer.m_field.setRobotPose(m_robotContainer.drivetrain.getState().Pose);

    }

    @Override
    public void disabledInit() {}

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
