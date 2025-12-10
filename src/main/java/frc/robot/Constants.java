
package frc.robot;

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

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(15);
  public static final double MAX_CREEP_SPEED = Units.feetToMeters(5); // 3 for cookie bot
  public static final double MAX_ANGULAR_VELOCITY = 5.627209491911525; // radians per second
  public static final double MAX_CREEP_ANGULAR_VELOCITY = 3; // 1 for cookie bot
  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

public final class Robot {
  public static final double width = 0.1; // metres
  public static final double leftArmActivationMinHeight =
      0.45; // metres from position on robot camera positions are based on
  public static final double rightArmActivationMinHeight = 0.40;
  public static final double rightArmActivationMaxHeight = 0.60;
  public static final double leftArmActivationMaxHeight = 1.00;
  //
  public static final double armActivationMaxHeight = 0.60;
  public static final double legActivationMaxHeight = 0.30;
  public static final double SlideDistance = 0.1;
  public static final double GravityEffect = 0.15;
  public static final double secondsBeforeSave = 5.0;
}

  public static final class DrivebaseConstants{

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants{

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }

  public static class CAN_IDS{
    
    public static final int LEFT_LEG_MOTOR = 3;
    public static final int RIGHT_LEG_MOTOR = 4;
    public static final int LEFT_ARM_MOTOR = 5;
    public static final int RIGHT_ARM_MOTOR = 6;
    public static final int HEAD_ID = 7;
    public static final int LIGHTS = 0; // PWM
  }

  public static class Head{
    public class PID {
      public static final double P = 0.9;
      public static final double I = 0;
      public static final double D = 0.08;
    }

    public final class Positions {
      public static final double TOP_POSITION = 14;
      public static final double BOTTOM_POSITION = 2;
    }

    public static final int TOP_LIMIT_SWITCH_ID = 0;
    public static final int BOTTOM_LIMIT_SWITCH_ID = 1;
    public static final int LEFT_LIMIT_SWITCH_ID = 2;
    public static final int RIGHT_LIMIT_SWITCH_ID = 3;
    public static final int STEP_PIN = 4;
    public static final int DIR_PIN = 5;
  }

  public final class Arms {

    public class RightPID {
      public static final double P = 0.035;
      public static final double I = 0;
      public static final double D = 0.004;
    }

    public class LeftPID {
      public static final double P = 0.03;
      public static final double I = 0.00;
      public static final double D = 0.07;
    }

    public class Positions {
      public static final double leftMaxSavePosition = 231; // 206
      public static final double leftMinSavePosition = 160; // 157

      public static final double rightMaxSavePosition = 240; // 188
      public static final double rightMinSavePosition = 161; // 232

      public static final double upperStopRange = 1;
      public static final double lowerStopRange = 5;

      public static final double leftMaxPosition = 235;
      public static final double leftMinPosition = 155;
    }

    public final class motorControllerConfigurations {
      public static final int currentLimit = 40;
    }
  }

  public final class Legs{
    public class leftPID {
      public static final double P = 0.007; // 0.009
      public static final double I = 0; // 0.003
      public static final double D = 0.002; //0.002
    }

    public class rightPID {
      public static final double P = 0.012;
      public static final double I = 0.003;
      public static final double D = 0.002;
    }

    public class Positions {
      public static final double leftUpPosition = 239;
      public static final double leftDownPosition = 170;

      public static final double rightUpPosition = 228;
      public static final double rightDownPosition = 164;

      public static final double upperStopRange = 0;
      public static final double lowerStopRange = 5;
    }

    public final class motorControllerConfigurations {
      public static final int currentLimit = 30;
    }

  }

  public final class ButtonBoxConfig{
    public static final int port = 1;
      public static final int zeroHeadButton = 1;
      public static final int danceButton = 3;
      public static final int waveButton = 2;
      public static final int clearCameraDataButton = 4;
      public static final int toggleHeadButton = 5;
      public static final int rightLegToggleButton = 7;
      public static final int leftLegToggleButton = 6;
      public static final int enableAutoSaveButton = 8;
      public static final int enableMotorsSwitch = 10;
      public static final int manualModeSwitch = 9;
      public static final int invertButtonBoxSwitch = 11;
  }

  public final class Vision {
    public final class UpperCamera {
      public static final String address = "UpperFrontCamera";
      public static final double xOffset = 0;
      public static final double yOffset = 0.89;
      public static final double upTilt = -5;
      public static final double rightTilt = -4;
    }

    public final class LowerCamera {
      public static final String address = "LowerFrontCamera";
      public static final double xOffset = 0;
      public static final double yOffset = -.13;
      public static final double upTilt = 15;
      public static final double rightTilt = -1;
    }

    public final class BackCamera {
      public static final String address = "BackCamera";
    }
  }
}