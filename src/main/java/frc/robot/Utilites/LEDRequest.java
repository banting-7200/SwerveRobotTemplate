package frc.robot.Utilites;

import edu.wpi.first.wpilibj.util.Color;

public class LEDRequest {

    public enum LEDState {
        BLINK,
        SOLID,
        OFF,
        RAINBOW;
    };

    private LEDState state;
    private int blinkRate = 0;
    private Color color = Color.kBlack;
    private int priority = 0;

    public LEDRequest(LEDState state) {
        this.state = state;
    }

    public LEDRequest withBlinkRate(int rate) {
        this.blinkRate = rate;
        return this;
    }

    public LEDRequest withColour(Color c) {
        this.color = c;
        return this;
    }

    public LEDRequest withPriority(int p) {
        this.priority = p;
        return this;
    }

    public LEDState getState() { return state; }
    public int getBlinkRate() { return blinkRate; }
    public Color getColour() { return color; }
    public int getPriority() { return priority; }
}
