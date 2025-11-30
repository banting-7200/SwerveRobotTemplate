package frc.robot.Subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Utilites.HelperFunctions;
import frc.robot.Utilites.Constants.CANIds;
import frc.robot.Utilites.Constants.SparkMaxConstants;

import com.revrobotics.spark.config.SparkMaxConfig;

public class SparkMaxSubsystem {

    SparkMax motor;
    SparkClosedLoopController PIDController;
    SparkMaxConfig config;
    SparkLimitSwitch forwardLimitSwitch;
    SparkLimitSwitch reverseLimitSwitch;
    IdleMode idleMode;
    double setpoint;

    public SparkMaxSubsystem() {
        motor = new SparkMax(CANIds.SPARKMAX_ID, MotorType.kBrushless);
        config = new SparkMaxConfig();
        PIDController = motor.getClosedLoopController();
        idleMode = IdleMode.kBrake;

        config.inverted(SparkMaxConstants.IS_INVERTED).idleMode(idleMode);
        config.absoluteEncoder.positionConversionFactor(360).velocityConversionFactor(1);
        config.closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pidf(SparkMaxConstants.P, SparkMaxConstants.I, SparkMaxConstants.D, SparkMaxConstants.FF);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        forwardLimitSwitch = motor.getForwardLimitSwitch();
        reverseLimitSwitch = motor.getReverseLimitSwitch();

    }

    public void setIdleMode(IdleMode idleMode) {
        this.idleMode = idleMode;
        config.idleMode(idleMode);
        motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public boolean withinSoftLimits() {
        return !(setpoint >= SparkMaxConstants.FORWARD_SOFT_LIMIT) || !(setpoint <= SparkMaxConstants.REVERSE_SOFT_LIMIT);
    }

    public void setReferencePoint(double setpoint){
        this.setpoint = setpoint;
    }

    public void run() {
        setpoint = HelperFunctions.clamp(setpoint, SparkMaxConstants.REVERSE_SOFT_LIMIT, SparkMaxConstants.FORWARD_SOFT_LIMIT);
        PIDController.setReference(setpoint, ControlType.kPosition);
    }



}
