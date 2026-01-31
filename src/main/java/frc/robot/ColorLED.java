package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.leds.AddressableLEDPattern;
import frc.robot.leds.ChasePattern;
import frc.robot.subsystems.leds;


/**
 * Manages multiple LED strips on one PWM port using buffer views.
 */
public class ColorLED {
    private AddressableLEDBuffer m_ledBuffer;
    public AddressableLED m_led;
    // public final leds m_leds = new leds();
    // Note: buffer views are necessary because the RIO only works with 1 LED strip, this will be unecessary when SystemCore is rolled out late 2026 2027
    private List<AddressableLEDBufferView> ledBuffers;

    private int m_rainbowFirstPixelHue;
    // private Color[] redWhiteArray = {Color.kBlueViolet, Color.kBlue};
    // private Color[] blueWhiteArray = {Color.kBlue, Color.kWhite};
    // private AddressableLEDPattern m_redChasePattern = new ChasePattern(redWhiteArray, 4);

    /**
     * Creates LED strips from lengths array.
     * @param port PWM port
     * @param lengths array of LED counts per strip
     */
    public ColorLED(int port, int[] lengths) {
        int totalLength = 0;
        for (int len : lengths) totalLength += len;
        
        m_led = new AddressableLED(port);
        m_ledBuffer = new AddressableLEDBuffer(totalLength);
        //144 lights on big strand
        //75 seems to be max amount before something explodes
        //8 lights on small strand
        m_led.setLength(m_ledBuffer.getLength());

        m_led.setData(m_ledBuffer);

        m_led.start();

        ledBuffers = new ArrayList<>();
        int start = 0;
        for (int len : lengths) {
            int end = start + len - 1;
            ledBuffers.add(m_ledBuffer.createView(start, end));
            start += len;
        }
    }

    // public static final int INTAKE = 0; // First 8 LEDs (0-7)
    // public static final int SHOOTER = 1; // Second 8 LEDs (8-15)
    // public static final int CLIMB = 2; // Third 8 LEDs (16-23)
    
    // private static final int LEDS_PER_STRIP = 8;

    // public void setLED(int stripIndex, int red, int green, int blue) {
    //     // Clamp values to valid range
    //     int r = Math.max(0, Math.min(255, red));
    //     int g = Math.max(0, Math.min(255, green));
    //     int b = Math.max(0, Math.min(255, blue));

    //     int startIndex = stripIndex * LEDS_PER_STRIP;

    //     for (int i = 0; i < LEDS_PER_STRIP; i++) {
    //         int bufferIndex = startIndex + i;
            

    //         if (bufferIndex < m_ledBuffer.getLength()) {
    //             m_ledBuffer.setRGB(bufferIndex, r, g, b);
    //         }
    //     }
    //     m_led.setData(m_ledBuffer);
    // }


    // ==========================================================================================================================================================================================
    //     available: red, blue, green, orange, yellow, purple, white, gold, rainbow, tomfoolery(bouncing[nofunctionality])
        

        // m_redChasePattern.setLEDs(m_ledBuffer);

        // // Set the LEDs

        // m_led.setData(m_ledBuffer);
    // ==========================================================================================================================================================================================

     public void none() {

        // For every pixel
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            m_ledBuffer.setRGB(i, 0, 0, 0);
        }
        System.out.print("none");
        m_led.setData(m_ledBuffer);
    }



    public void rainbow() {
        // For every pixel
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
           // Calculate the hue - hue is easier for rainbows because the color
             // shape is a circle so only one value needs to precess
              var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;

             // Set the value
             m_ledBuffer.setHSV(i, hue, 255, 128);
        }
        // Increase by to make the rainbow "move"
        m_rainbowFirstPixelHue += 6;
            
        // Check bounds
        m_rainbowFirstPixelHue %= 180;
        m_led.setData(m_ledBuffer);
    }
     
    public void red() {
        
        // For every pixel
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            m_ledBuffer.setRGB(i, 255, 0, 0);
        }
        
        System.out.print("red");
        m_led.setData(m_ledBuffer);
    }

    public void orange() {

        // For every pixel
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            m_ledBuffer.setRGB(i, 255, 165, 0);
         }
         System.out.print("orange");
         m_led.setData(m_ledBuffer);
    }

    public void yellow() {

        // For every pixel
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            m_ledBuffer.setRGB(i, 255, 255, 0);
         }
         System.out.print("yellow");
         m_led.setData(m_ledBuffer);
    }
    
    public void green() {

        // For every pixel
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            m_ledBuffer.setRGB(i, 0, 255, 0);
        }
        System.out.print("green");
        m_led.setData(m_ledBuffer);
    }
        
    public void blue() {

        // For every pixel
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            m_ledBuffer.setRGB(i, 0, 0, 255);
        }
        System.out.print("blue");
        m_led.setData(m_ledBuffer);
    }

    public void purple() {

        // For every pixel
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            m_ledBuffer.setRGB(i, 255, 0, 255);
         }
         System.out.print("purple");
         m_led.setData(m_ledBuffer);
    }
    
    public void white() {
        
        // For every pixel
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            m_ledBuffer.setRGB(i, 255, 255, 255);
        }
        System.out.print("white");
        m_led.setData(m_ledBuffer);
    }
    
    public void gold() {

        // For every pixel
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            m_ledBuffer.setRGB(i, 255, 215, 0);
         }
         System.out.print("gold");
         m_led.setData(m_ledBuffer);
    }

    public void tomfoolery() {

        // For every pixel
        //Color x = new Color(0,0,0);
        for (var i = 1; i <= 140; i++) {
            if (i%20==0 && i>0) {
                System.out.println("i="+ i);
                m_ledBuffer.setRGB(i/20-3
                , 0, 0, 0);
                m_ledBuffer.setRGB(i/20, 255, 215, 0);                       
                m_led.setData(m_ledBuffer);
            }
            //m_ledBuffer.setHSV(i, hue, 0, 128);
//https://github.com/FRC-5013-Park-Hill-Robotics/5013-RapidReact/blob/main/src/main/java/frc/robot/trobot5013lib/led/ChasePattern.java
            // Calculate the hue - hue is easier for rainbows because the color

            // shape is a circle so only one value needs to precess

            //final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;

            // Set the value

            
           

        }

        // Increase by to make the rainbow "move"

        m_rainbowFirstPixelHue += 3;

        // Check bounds

        m_rainbowFirstPixelHue %= 180;

    }
}
