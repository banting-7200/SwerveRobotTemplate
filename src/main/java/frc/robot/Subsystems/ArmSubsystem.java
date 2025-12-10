package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.fasterxml.jackson.annotation.JsonTypeInfo.Id;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Arms;

public class ArmSubsystem extends SubsystemBase {

    SparkMax motor;
    SparkClosedLoopController pidController;
    SparkMaxConfig config;
    ClosedLoopConfig pidConfig;

    private double currentPosition;
    private double setPosition;

    private double upPosition;
    private double downPosition;
    private double upWavePosition = Arms.Positions.leftMaxPosition;

    private double upperStopRange = Arms.Positions.upperStopRange;
    private double lowerStopRange = Arms.Positions.lowerStopRange;

    private boolean enabled = false;
    private boolean isArmUp = false;

    public ArmSubsystem(int deviceID, double upPosition, double downPosition, boolean isInverted) {

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
        config.smartCurrentLimit(Arms.motorControllerConfigurations.currentLimit);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pidConfig = config.closedLoop;

    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    public boolean isEnabled() {
        return enabled;
    }

    public void run() {
        currentPosition = motor.getAbsoluteEncoder().getPosition();
        if (true) { // if enabled
            if (setPosition > currentPosition) {
                if (!withinUpperLimits()) {
                     System.out.println("Within Upper Arm Limit");
                    return;
                }
            } else if (!withinLowerLimits()) {
                 System.out.println("Within Lower Arm Limit");
                return;
            } 
            //System.out.println("MOVING LEFT ARM, Current Position: " + currentPosition + " Setpoint: " + setPosition);
            pidController.setReference(setPosition, ControlType.kPosition);
        } else {
            motor.set(0);
        }
    }

    public void toggleArmPosition() {
        isArmUp = !isArmUp;
        if (isArmUp) {
            moveToAngle(upPosition);
        } else {
            moveToAngle(downPosition);
        }
    }

    private boolean withinUpperLimits() {
        return (currentPosition < upPosition + upperStopRange);
    }

    private boolean withinLowerLimits() {
        return (currentPosition > downPosition - lowerStopRange);
    }

    /**
     * Moves the arm to a position based on where an input is within a given range.
     *
     * @param rangeMax the maximum value of the range
     * @param rangeMin the minimum value of the range
     * @param input    the value within this range to map the arm position to
     */
    public void moveFromRange(double rangeMin, double rangeMax, double input) {
        if (input > rangeMax)
            input = rangeMax;
        if (input < rangeMin)
            input = rangeMin;

        double position = (input - rangeMin) / (rangeMax - rangeMin) * (upPosition - downPosition) + downPosition;

        moveToAngle(position);
    }

    public void moveFromRangeWave(double rangeMin, double rangeMax, double input) {
        if (input > rangeMax)
            input = rangeMax;
        if (input < rangeMin)
            input = rangeMin;

        double position = (input - rangeMin) / (rangeMax - rangeMin) * (upWavePosition - (upPosition - 30))
                + (upPosition - 30);

        moveToAngle(position);
    }

    public void moveToAngle(double setPosition) {
        this.setPosition = setPosition;
    }

    public double getPosition() {
        return currentPosition;
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

    public double[] getPID() {
        return new double[] { 0, 0, 0 };
    }

    public double getCurrent() {
        return motor.getOutputCurrent();
    }

    public void moveToUpPosition() {
        moveToAngle(upPosition);
    }

    public void moveToDownPosition() {
        moveToAngle(setPosition);
    }
}
