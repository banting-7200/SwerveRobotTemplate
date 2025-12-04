package frc.robot.Subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Legs;

public class LegSubsystem extends SubsystemBase {

    SparkMax motor;
    SparkClosedLoopController pidController;
    SparkMaxConfig config;
    ClosedLoopConfig pidConfig;

    private double upperStopRange = Legs.Positions.upperStopRange;
    private double lowerStopRange = Legs.Positions.lowerStopRange;

    private double upPosition;
    private double midPosition;
    private double downPosition;
    private double currentPosition;
    private double setPosition;

    private boolean isUp = true;

    public LegSubsystem(int deviceID, double upPosition, double downPosition, boolean isInverted) {

        config = new SparkMaxConfig();
        this.upPosition = upPosition;
        this.downPosition = downPosition;
        setPosition = downPosition;
        motor = new SparkMax(deviceID, MotorType.kBrushless);
        pidController = motor.getClosedLoopController();
        config.inverted(isInverted).idleMode(IdleMode.kBrake);
        config.absoluteEncoder.positionConversionFactor(360).velocityConversionFactor(1);
        config.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder).pid(0, 0, 0);
        config.limitSwitch.forwardLimitSwitchType(Type.kNormallyClosed).reverseLimitSwitchType(Type.kNormallyClosed);
        config.smartCurrentLimit(30);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pidConfig = config.closedLoop;
    }

    public void togglePosition() {
        isUp = !isUp;
        if (isUp) {
            setPosition(upPosition);
        } else {
            setPosition(downPosition);
        }
    }

    public void moveToUpPosition() {
        isUp = true;
        setPosition(upPosition);
    }

    public void moveToMidPosition() {
        setPosition(midPosition);
    }

    public void moveToDownPosition() {
        isUp = false;
        setPosition(downPosition);
    }

    public void moveFromRange(double rangeMin, double rangeMax, double input) {
        if (input > rangeMax)
            input = rangeMax;
        if (input < rangeMin)
            input = rangeMin;

        double position = (input - rangeMin) / (rangeMax - rangeMin) * (upPosition - downPosition) + downPosition;

        setPosition(position);
    }

    public void setPosition(double setPosition) {
        this.setPosition = setPosition;
    }

    public double getCurrentsetPosition() {
        return setPosition;
    }

    public double getPosition() {
        return motor.getAbsoluteEncoder().getPosition();
    }

    public void run() {
        currentPosition = getPosition();
        if (isUp) {
            if (withinUpperSoftLimits()) {
                pidController.setReference(setPosition, ControlType.kPosition);
            } else {
                motor.set(0);
            }
        } else if (!isUp) {
            if (withinLowerSoftLimits()) {
                pidController.setReference(setPosition, ControlType.kPosition);
            } else {
                motor.set(0);
            }
        } else {
            motor.set(0);
        }
    }

    private boolean withinLowerSoftLimits() {
        // System.out.println(
        // "LOWER SOFT LIMITS "
        // + downPosition
        // + " || "
        // + currentPosition
        // + " || "
        // + upPosition);
        return (currentPosition > downPosition + lowerStopRange);
    }

    private boolean withinUpperSoftLimits() {
        // System.out.println(
        // "UPPER SOFT LIMITS "
        // + downPosition
        // + " || "
        // + currentPosition
        // + " || "
        // + upPosition);
        return (currentPosition < upPosition - upperStopRange);
    }

    public boolean isUp() {
        return isUp;
    }

    public void setPID(double P, double I, double D) {
        config.closedLoop.p(P);
        config.closedLoop.i(I);
        config.closedLoop.d(D);
        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
      }
    
      public void setPID(double[] PID) {
        config.closedLoop.p(PID[0]);
        config.closedLoop.i(PID[1]);
        config.closedLoop.d(PID[2]);
        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
      }

}
