package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;


/**
 * Manages multiple LED strips on one PWM port using buffer views.
 */
public class ColorLED {
    private AddressableLEDBuffer m_ledBuffer;
    private final Timer m_timer = new Timer();
    public AddressableLED m_led;
    // Note: buffer views are necessary because the RIO only works with 1 LED strip, this will be unecessary when SystemCore is rolled out late 2026 2027
    private List<AddressableLEDBufferView> ledBuffers;

    private int m_rainbowFirstPixelHue;
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

    // ====================================================================================================================================================================================================
    //     Available: none, rainbow, fasterfaster, climbprogressbar(not tested), blueflashing, greenflashing, red, maroon, orange, yellow, green, lime, blue, lightblue, purple, pink, magenta, white, gold
    // ====================================================================================================================================================================================================

     public Color getCurrentColor() {
        Color firstLEDColor = m_ledBuffer.getLED(0);
        return firstLEDColor;
     }

     public void none() {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, 0, 0, 0);
        }
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
        m_rainbowFirstPixelHue += 3; //higher number = faster
        // Check bounds
        m_rainbowFirstPixelHue %= 180;
        m_led.setData(m_ledBuffer);
    }

    public void fasterfaster() {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
            m_ledBuffer.setHSV(i, hue, 255, 128);
        }
        m_rainbowFirstPixelHue += 6;
        m_rainbowFirstPixelHue %= 180;
        m_led.setData(m_ledBuffer);
    }

    public void climbProgressBar() {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            var progress = (Robot.getInstance().m_climb.getRaiserPositionAsInt() / Robot.getInstance().m_climb.getRaiserMaxHeightAsInt() * 180) % 180;
            m_ledBuffer.setHSV(i, progress, 255, 128);
        }
        m_rainbowFirstPixelHue %= 180;
        m_led.setData(m_ledBuffer);
    }

    public void blueFlashing() {
        m_timer.start();
        if (m_timer.get() > 0.25) {
            for (var i = 0; i < m_ledBuffer.getLength(); i++) {
                m_ledBuffer.setRGB(i, 0, 0, 0);
            }
            while (m_timer.get() > 0.5) {
                m_timer.reset();
            }
        }
        else if (m_timer.get() <= 0.25) {
            for (var i = 0; i < m_ledBuffer.getLength(); i++) {
                m_ledBuffer.setRGB(i, 0, 0, 255);
            }
            while (m_timer.get() > 1) {
                m_timer.reset();
            }
        }
        m_led.setData(m_ledBuffer);
    }

    public void greenFlashing() {
        m_timer.start();
        if (m_timer.get() > 0.25) {
            for (var i = 0; i < m_ledBuffer.getLength(); i++) {
                m_ledBuffer.setRGB(i, 0, 0, 0);
            }
            while (m_timer.get() > 0.5) {
                m_timer.reset();
            }
        }
        else if (m_timer.get() <= 0.25) {
            for (var i = 0; i < m_ledBuffer.getLength(); i++) {
                m_ledBuffer.setRGB(i, 17, 125, 0);
            }
            while (m_timer.get() > 1) {
                m_timer.reset();
            }
        }
        m_led.setData(m_ledBuffer);
    }
     
    public void red() {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, 255, 0, 0);
        }
        
        m_led.setData(m_ledBuffer);
    }

    public void maroon() {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, 128, 0, 0);
        }
        
        m_led.setData(m_ledBuffer);
    }

    public void orange() {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, 255, 165, 0);
         }
         m_led.setData(m_ledBuffer);
    }

    public void yellow() {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, 255, 255, 0);
         }
         m_led.setData(m_ledBuffer);
    }
    
    public void green() {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, 17, 125, 0);
        }
        m_led.setData(m_ledBuffer);
    }
    
    public void lime() {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, 0, 255, 0);
        }
        m_led.setData(m_ledBuffer);
    }
      
    public void blue() {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, 0, 0, 255);
        }
        m_led.setData(m_ledBuffer);
    }

    public void lightblue() {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, 0, 255, 255);
        }
        m_led.setData(m_ledBuffer);
    }

    public void purple() {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, 128, 0, 128);
         }
         m_led.setData(m_ledBuffer);
    }

    public void pink() {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, 255, 145, 255);
         }
         m_led.setData(m_ledBuffer);
    }

    public void magenta() {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, 255, 0, 255);
         }
         m_led.setData(m_ledBuffer);
    }
    
    public void white() {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, 255, 255, 255);
        }
        m_led.setData(m_ledBuffer);
    }
    
    public void gold() {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, 255, 215, 0);
         }
         m_led.setData(m_ledBuffer);
    }
}
