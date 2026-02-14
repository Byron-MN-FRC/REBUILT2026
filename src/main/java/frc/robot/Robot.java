// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
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

    public final Field2d field = new Field2d();

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
        SmartDashboard.putData("Robot Position", field);
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
    }
        
//         CommandScheduler.getInstance().run();
//         // Fill the buffer with a rainbow

//         //available: red, blue, green, orange, white, gold, rainbow, tomfoolery(bouncing[nofunctionality])
//         //set the name below to one of the values above
//         //rainbow();


//         //m_redChasePattern.setLEDs(m_ledBuffer);

//         // Set the LEDs

//         //m_led.setData(m_ledBuffer);
        


//         // buggedcode?

//         ShuffleboardTab ledTab = Shuffleboard.getTab("LEDs");
//     }

//     private void rainbow() {
//         // For every pixel
//         for (var i = 0; i < m_ledBuffer.getLength(); i++) {
//            // Calculate the hue - hue is easier for rainbows because the color
//              // shape is a circle so only one value needs to precess
//               var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;

//              // Set the value
//              m_ledBuffer.setHSV(i, hue, 255, 128);
//         }
//         // Increase by to make the rainbow "move"
//         m_rainbowFirstPixelHue += 6;
            
//         // Check bounds
//         m_rainbowFirstPixelHue %= 180;
//         m_led.setData(m_ledBuffer);

//     }
     
        
        
        
//     private void blue() {

//         // For every pixel
//         for (var i = 0; i < m_ledBuffer.getLength(); i++) {
//             // Sets the specified LED to the RGB values for red
//             m_ledBuffer.setRGB(i, 0, 0, 255);
//         }
//         System.out.print("blue");
//         m_led.setData(m_ledBuffer);
//     }
        
//     private void green() {

//         // For every pixel
//         for (var i = 0; i < m_ledBuffer.getLength(); i++) {
//             // Sets the specified LED to the RGB values for red
//             m_ledBuffer.setRGB(i, 0, 255, 0);
//         }
//         System.out.print("green");
//         m_led.setData(m_ledBuffer);
//     }
    
//     private void white() {
        
//         // For every pixel
//         for (var i = 0; i < m_ledBuffer.getLength(); i++) {
//             // Sets the specified LED to the RGB values for red
//             m_ledBuffer.setRGB(i, 255, 255, 255);
//         }
//         System.out.print("white");
//         m_led.setData(m_ledBuffer);
//     }
    
//     private void red() {
        
//         // For every pixel
//         for (var i = 0; i < m_ledBuffer.getLength(); i++) {
//             // Sets the specified LED to the RGB values for red
//             m_ledBuffer.setRGB(i, 255, 0, 0);
//         }
        
//         System.out.print("red");
//         m_led.setData(m_ledBuffer);
//     }

//     private void orange() {

//         // For every pixel
//         for (var i = 0; i < m_ledBuffer.getLength(); i++) {
//             // Sets the specified LED to the RGB values for red
//             m_ledBuffer.setRGB(i, 255, 165, 0);
//          }
//          System.out.print("orange");
//          m_led.setData(m_ledBuffer);
//     }

//     private void gold() {

//         // For every pixel
//         for (var i = 0; i < m_ledBuffer.getLength(); i++) {
//             // Sets the specified LED to the RGB values for red
//             m_ledBuffer.setRGB(i, 255, 215, 0);
//          }
//          System.out.print("gold");
//          m_led.setData(m_ledBuffer);
//     }




//     private void tomfoolery() {

//         // For every pixel
//         //Color x = new Color(0,0,0);
//         for (var i = 1; i <= 140; i++) {
//             if (i%20==0 && i>0) {
//                 System.out.println("i="+ i);
//                 m_ledBuffer.setRGB(i/20-3
//                 , 0, 0, 0);
//                 m_ledBuffer.setRGB(i/20, 255, 215, 0);                       
//                 m_led.setData(m_ledBuffer);
//             }
//             //m_ledBuffer.setHSV(i, hue, 0, 128);
// //https://github.com/FRC-5013-Park-Hill-Robotics/5013-RapidReact/blob/main/src/main/java/frc/robot/trobot5013lib/led/ChasePattern.java
//             // Calculate the hue - hue is easier for rainbows because the color

//             // shape is a circle so only one value needs to precess

//             //final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;

//             // Set the value

            
           

//         }

//         // Increase by to make the rainbow "move"

//         m_rainbowFirstPixelHue += 3;

//         // Check bounds

//         m_rainbowFirstPixelHue %= 180;

//     }

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
