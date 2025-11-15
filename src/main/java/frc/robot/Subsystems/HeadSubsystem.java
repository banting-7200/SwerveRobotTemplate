package frc.robot.Subsystems;

import java.util.concurrent.locks.Condition;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.Head;

public class HeadSubsystem {

    TalonFX motor;
    PositionVoltage positionRequest;
    DigitalInput bottomLimitSwitch;
    DigitalInput topLimitSwitch;
    DigitalInput leftLimitSwitch;
    DigitalInput rightLimitSwitch;
    DigitalOutput stepperStepPIN;
    DigitalOutput stepperDirPIN;

    boolean enabledMovement;
    boolean atUpPosition = false;
    boolean isZeroed = false;
    boolean steppingState = false;

    double setpoint = 0;
    double angleSetpoint = 0;
    double currentPosition = 0;
    double lastStepTime = 0;
    double stepInterval = 0.002; // 2ms
    double currentAngle = 0;

    int stepsRemaining = 0;

    public HeadSubsystem() {
        bottomLimitSwitch = new DigitalInput(Head.BOTTOM_LIMIT_SWITCH_ID);
        topLimitSwitch = new DigitalInput(Head.TOP_LIMIT_SWITCH_ID);
        leftLimitSwitch = new DigitalInput(Head.LEFT_LIMIT_SWITCH_ID);
        rightLimitSwitch = new DigitalInput(Head.RIGHT_LIMIT_SWITCH_ID);
        stepperStepPIN = new DigitalOutput(Head.STEP_PIN);
        stepperDirPIN = new DigitalOutput(Head.DIR_PIN);

        motor = new TalonFX(Constants.CAN_IDS.HEAD_ID, "rio");
        TalonFXConfiguration configs = new TalonFXConfiguration();
        positionRequest = new PositionVoltage(0).withSlot(0);

        configs.MotorOutput.withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Coast);
        configs.Slot0.kP = Head.PID.P;
        configs.Slot0.kI = Head.PID.I;
        configs.Slot0.kD = Head.PID.D;
        configs.CurrentLimits.withSupplyCurrentLimit(20);
        configs.CurrentLimits.withStatorCurrentLimit(20);


        motor.getConfigurator().apply(configs);

        positionRequest = new PositionVoltage(0);
    }

    public void enableMovement(boolean enabledMovement) {
        this.enabledMovement = enabledMovement;
    }

    public boolean isEnabled() {
        return enabledMovement;
    }

    public boolean withinVerticalLimits() {
        return (topLimitSwitch.get());
    }

    public boolean withinHorizontalLimits() {
        return (leftLimitSwitch.get() && rightLimitSwitch.get());
    }

    public void toggleHead() {
        atUpPosition = !atUpPosition;
        setpoint = atUpPosition ? Head.Positions.TOP_POSITION : Head.Positions.BOTTOM_POSITION;
    }

    public void setHeadPosition(double rangeMin, double rangeMax, double input) {
        double position = (input - rangeMin)
                / (rangeMax - rangeMin)
                * (Head.Positions.TOP_POSITION - Head.Positions.BOTTOM_POSITION)
                + Head.Positions.BOTTOM_POSITION;
        setpoint = position;
    }

    public double getCurrentPosition() {
        currentPosition = motor.getPosition().getValueAsDouble();
        return currentPosition;
    }

    public void reZero() {
        isZeroed = false;
    }

    public void zeroEncoderPeriodic() {
        if (isZeroed)
            return;

        if (bottomLimitSwitch.get()) {
            motor.set(-0.06);
           // System.out.println(motor.getPosition());
        } else {
            motor.set(0);
           // System.out.println("Head Zeroed at position: " + motor.getPosition().getValueAsDouble());
            motor.setPosition(0);
            setpoint = Head.Positions.BOTTOM_POSITION;
            atUpPosition = false;
            isZeroed = true;
            positionRequest.Position = setpoint;
            //motor.setControl(positionRequest);
        }
    }

    public void run() {
        if(!isZeroed){
            return;
        }
        if (withinVerticalLimits()) {
           // System.out.println("Current Position: " + motor.getPosition().getValueAsDouble() + " Setpoint: " + setpoint);
            motor.setControl(new PositionVoltage(setpoint));
        } else{
            setpoint = currentPosition;
            motor.set(0);
        }

        // if (withinHorizontalLimits()) {
        //     if (stepsRemaining > 0) {
        //         double now = Timer.getFPGATimestamp();
        //         if (now - lastStepTime >= stepInterval) {
        //             // Toggle step pin
        //             steppingState = !steppingState;
        //             stepperStepPIN.set(steppingState);

        //             // Only count a step on rising edge
        //             if (steppingState) {
        //                 stepsRemaining--;
        //             }

        //             lastStepTime = now;
        //         }
        //     }
        // }

    }

    public void setDirection(boolean isClockwise) {
        stepperDirPIN.set(isClockwise);
    }

    public void moveSteps(int steps) {
        stepsRemaining = steps;
        lastStepTime = Timer.getFPGATimestamp();
    }

    public void setAngle(double setpoint) {
        // 0 to 100

        if (angleSetpoint > currentAngle) {
            setDirection(true); // rotate right
        } else if (angleSetpoint < currentAngle) {
            setDirection(false); // rotate left
        }

        moveSteps((int) Math.abs(currentAngle - angleSetpoint));
    }

    public void rotateLeft() {
        System.out.println(leftLimitSwitch.get());
    }
}