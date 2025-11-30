package frc.robot.Utilites;

import edu.wpi.first.math.MathUtil;

public class HelperFunctions {

    public static double clamp(double value, double min, double max) {
        return MathUtil.clamp(value, min, max);
    }

    public static double map(double input, double inMin, double inMax, double outMin, double outMax) {
        return (input - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    }
    

}
