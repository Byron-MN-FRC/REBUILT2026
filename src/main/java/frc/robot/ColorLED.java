package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.util.Color;
import java.util.ArrayList;
import java.util.List;

/**
 * Manages multiple LED strips on one PWM port using buffer views.
 */
public class ColorLED {
    private AddressableLEDBuffer m_ledBuffer;
    public AddressableLED m_led;
    // Note: buffer views are necessary because the RIO only works with 1 LED strip, this will be unecessary when SystemCore is rolled out late 2026 2027
    private List<AddressableLEDBufferView> ledBuffers;

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

    public static final int INTAKE = 0; // First 8 LEDs (0-7)
    public static final int SHOOTER = 1; // Second 8 LEDs (8-15)
    public static final int CLIMB = 2; // Third 8 LEDs (16-23)
    
    private static final int LEDS_PER_STRIP = 8;

    public void setLED(int stripIndex, int red, int green, int blue) {
        // Clamp values to valid range
        int r = Math.max(0, Math.min(255, red));
        int g = Math.max(0, Math.min(255, green));
        int b = Math.max(0, Math.min(255, blue));

        int startIndex = stripIndex * LEDS_PER_STRIP;

        for (int i = 0; i < LEDS_PER_STRIP; i++) {
            int bufferIndex = startIndex + i;
            

            if (bufferIndex < m_ledBuffer.getLength()) {
                m_ledBuffer.setRGB(bufferIndex, r, g, b);
            }
        }
        m_led.setData(m_ledBuffer);
    }
}
