package frc.robot.Subsystems;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utilites.LEDRequest;

//LED PRIORITY LIST

// TODO Implement Elastic notifications
// -1 - Error (Look at dashboard Notification)

// 0 - Robot disabled

// 1 - Has gamePiece / Done command

// 2 - Aligning to aprilTag/GamePiece

// 3 - Ready to do something

// 4 - Creep Drive mode

// 5 - Normal driving

public class LightsSubsystem extends SubsystemBase {


    private List<LEDRequest> requests = new ArrayList<>();
    private LEDRequest currentRequest = new LEDRequest(LEDRequest.LEDState.OFF).withBlinkRate(0)
            .withColour(Color.kWhite).withPriority(Integer.MAX_VALUE);
    private AddressableLED ledInstance;
    private AddressableLEDBuffer bufferInstance;
    private int rainbowFirstPixelHue = 0;
    private double lastReadTimestamp = Timer.getFPGATimestamp();

    public LightsSubsystem(int lightPort, int lightCount) {
        ledInstance = new AddressableLED(lightPort);
        bufferInstance = new AddressableLEDBuffer(lightCount);
        ledInstance.setLength(lightCount);
        ledInstance.start();
    }

    public void requestLEDState(LEDRequest newRequest) {
        requests.add(newRequest);
    }

    public void run() {

        if(requests.isEmpty()){
            off();
        } else if(requests.size() == 1){
            currentRequest = requests.get(0);
        } else{
            // no idea if this works, get the LEDState with the lowest int for Priority
            currentRequest = requests.stream().min(Comparator.comparingInt(LEDRequest::getPriority)).orElse(requests.get(0));
        }

        switch (currentRequest.getState()) {
            case OFF:
                off();
                break;
            case SOLID:
                solidColor(currentRequest.getColour());
                break;
            case BLINK:
                blink(currentRequest.getBlinkRate(), currentRequest.getColour());
                break;
            case RAINBOW:
                rainbow();
        }
        requests.clear();
    }

    private void blink(int blinkRate, Color color) {
        off();
        double now = Timer.getFPGATimestamp();
        if (now - lastReadTimestamp > blinkRate) {
            lastReadTimestamp = now;
            for (int x = 0; x < bufferInstance.getLength(); x++) {
                bufferInstance.setLED(x, color);
            }
        }
        ledInstance.setData(bufferInstance);
    }

    private void solidColor(Color color) {
        for (int x = 0; x < bufferInstance.getLength(); x++) {
            bufferInstance.setLED(x, color);
        }
        ledInstance.setData(bufferInstance);
    }

    private void rainbow() {
        for (var i = 0; i < bufferInstance.getLength(); i++) {
            int hue = (rainbowFirstPixelHue + (i * 180 / bufferInstance.getLength())) % 180;
            bufferInstance.setHSV(i, hue, 255, 128);
        }
        rainbowFirstPixelHue += 3;
        rainbowFirstPixelHue %= 180;
        ledInstance.setData(bufferInstance);
    }

    private void off() {
        for (int i = 0; i < bufferInstance.getLength(); i++) {
            bufferInstance.setLED(i, Color.kBlack);
        }
        ledInstance.setData(bufferInstance);
    }
}
