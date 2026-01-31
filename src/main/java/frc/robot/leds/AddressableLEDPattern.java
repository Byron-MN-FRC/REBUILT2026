package frc.robot.leds;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

/** Add your docs here. */
public interface AddressableLEDPattern {
	public void setLEDs(AddressableLEDBuffer buffer);

	default boolean isAnimated() {
		return false;
	}
}