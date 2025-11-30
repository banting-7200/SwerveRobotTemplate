
package frc.robot.Utilites;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 */
public final class Constants{

  public static final double ROBOT_MASS = 63 * 0.453592; // 63lbs to kg
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);
  public static final double MAX_CREEP_SPEED = Units.feetToMeters(5); // 3 for cookie bot
  public static final double MAX_ANGULAR_VELOCITY = 5.627209491911525; // radians per second
  public static final double MAX_CREEP_ANGULAR_VELOCITY = 3; // 1 for cookie bot
  // Maximum speed of the robot in meters per second, used to limit acceleration.

  public static final class DrivebaseConstants{
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants{
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }

  public static final class PWMports{
    public static final int LIGHT_PORT = 0;
  }
  
}