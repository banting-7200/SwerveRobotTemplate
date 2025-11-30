package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Utilites.LEDRequest;

public class LightsSubsystem {

    
    private AddressableLED ledInstance;
    private AddressableLEDBuffer bufferInstance;
    private int rainbowFirstPixelHue = 0;

    public LightsSubsystem(int lightPort, int lightCount) {
        ledInstance = new AddressableLED(lightPort);
        bufferInstance = new AddressableLEDBuffer(lightCount);
        ledInstance.setLength(lightCount);
        ledInstance.start();
    }

    private LEDRequest currentRequest = new LEDRequest(LEDRequest.LEDState.OFF);

    public void requestLEDState(LEDRequest newRequest) {
        // Light state with lower priority goes
        if (newRequest.getPriority() >= currentRequest.getPriority()) {
            currentRequest = newRequest;
        }
    }


    public void run() {
        ledInstance.setData(bufferInstance);

        switch (currentRequest.getState()) {
            case OFF: off(); break;
            case SOLID: solidColor(currentRequest.getColour()); break;
            case BLINK: blink(currentRequest.getBlinkRate(), currentRequest.getColour()); break;
            case RAINBOW: rainbow();
        }

    }

    private void blink(int blinkRate, Color color) {

    }

    private void solidColor(Color color) {
        // for (int x = 0; x < bufferInstance.getLength(); x++) {
        //     bufferInstance.setRGB(x, red, green, blue);
        // }
    }

    private void rainbow() {
        for (var i = 0; i < bufferInstance.getLength(); i++) {
            int hue = (rainbowFirstPixelHue + (i * 180 / bufferInstance.getLength())) % 180;
            bufferInstance.setHSV(i, hue, 255, 128);
        }

        rainbowFirstPixelHue += 3;
        rainbowFirstPixelHue %= 180;
    }

    private void off() {
        for (int i = 0; i < bufferInstance.getLength(); i++) {
            bufferInstance.setLED(i, Color.kBlack);
        }
    }
}
