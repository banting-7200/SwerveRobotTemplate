package frc.robot.Utilites;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.util.Color;

public class HelperFunctions {

    public static double clamp(double value, double min, double max) {
        return MathUtil.clamp(value, min, max);
    }

    public static double map(double input, double inMin, double inMax, double outMin, double outMax) {
        return (input - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    }

    public static Color convertToGRB(Color rgbColor){
        return new Color(rgbColor.green, rgbColor.red, rgbColor.blue);
    }
    

}
