
package frc.robot.Utilites;

import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be
 * declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 */
public final class Constants {

  public static final int XBOX_PORT = 0;
  public static final int BUTTON_BOX_PORT = 1;
  public static final int LIGHTS_AMOUNT = 50;

  public static final class DrivebaseConstants {
    public static final double ROBOT_MASS = 63 * 0.453592; // 63lbs to kg
    public static final double MAX_SPEED = Units.feetToMeters(14.5);
    public static final double MAX_CREEP_SPEED = Units.feetToMeters(5);
    public static final double MAX_ANGULAR_VELOCITY = 5.627209491911525; // radians per second
    public static final double MAX_CREEP_ANGULAR_VELOCITY = 3;
    public static final double CRAZY_SPIN_SPEED = Units.feetToMeters(3);
  }

  public static class OperatorConstants {
    public static final double DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;
  }

  public static final class PWMPorts {
    public static final int LIGHT_PORT = 0;
  }

  public static final class DIOPorts {
    public static final int TALONFX_CW_LIMIT_SWITCH = 0;
    public static final int TALONFX_CCW_LIMIT_SWITCH = 1;
  }

  public static final class CANIds {
    public static final int SPARKMAX_ID = 1;
    public static final int TALONFX_ID = 2;
  }

  public static final class SparkMaxConstants{
    public static final double P = 0.0;
    public static final double I = 0.0;
    public static final double D = 0.0;
    public static final double FF = 0.0;
    public static final int CURRENT_LIMIT = 40;
    public static final boolean IS_INVERTED = false;
    public static final double FORWARD_SOFT_LIMIT = 0.0;
    public static final double REVERSE_SOFT_LIMIT = 0.0;
  }

  public static final class TalonFXConstants{
    public static final double P = 0.0;
    public static final double I = 0.0;
    public static final double D = 0.0;
    public static final double FF = 0.0;
    public static final int CURRENT_LIMIT = 40;
    public static InvertedValue invertedValue = InvertedValue.Clockwise_Positive;
    public static final double CW_SOFT_LIMIT = 0.0;
    public static final double CCW_SOFT_LIMIT = 0.0;
  }

}