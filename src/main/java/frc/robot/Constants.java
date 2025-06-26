package frc.robot;

import edu.wpi.first.math.util.Units;

public class Constants {
    
    public final class DriveBase {
        public static final double maxSpeed = Units.feetToMeters(15); // metres per second
        public static final double maxAngularVelocity = 5.627209491911525; // radians per second
        public static final double maxCreepSpeed = Units.feetToMeters(3); // metres per second
        public static final double maxMediumSpeed = Units.feetToMeters(8);
        public static final double maxCreepAngularVelocity = 2; // radians per second
        public static final double maxMediumAngularVelocity = 4;
        public static final double wheelDiameter = 4; // inches
        public static final double driveGearRatio = 6.75; // revolutions per wheel rotation
        public static final double angleGearRatio = 21.4285714286;
        public static final double anglePulsesPerRotation = 360;
    
        public final class TranslationPID {
          public static final double slowP = 0.2;
          public static final double normalP = 0.7;
          public static double p = 0.7;
    
          public static final double i = 0;
          public static final double d = 0;
        }
    
        public final class RotationPID {
          public static final double slowP = 0.2;
          public static final double normalP = 0.4;
          public static double p = 0.4;
    
          public static final double i = 0;
          public static final double d = 0;
        }
      }
}
