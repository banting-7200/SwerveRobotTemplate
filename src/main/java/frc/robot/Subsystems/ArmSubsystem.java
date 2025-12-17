package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.reduxrobotics.canand.ReduxJNI.Helper;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Utilites.HelperFunctions;
import frc.robot.Utilites.Constants.CANIds;
import frc.robot.Utilites.Constants.DIOPorts;
import frc.robot.Utilites.Constants.TalonFXConstants;

public class ArmSubsystem {

    TalonFX motor;
    NeutralModeValue neutralModeValue;
    DigitalInput CWLimitSwitch; // Rename to something related to mechanism ex. Top, left, max
    DigitalInput CCWLimitSwitch; // Counter-Clock wise

    boolean atUpPosition = false;
    boolean isZeroed = false;

    double setpoint = 0;
    double currentPosition = 0;

    public ArmSubsystem() {
        CWLimitSwitch = new DigitalInput(DIOPorts.TALONFX_CW_LIMIT_SWITCH);
        CCWLimitSwitch = new DigitalInput(DIOPorts.TALONFX_CCW_LIMIT_SWITCH);
        neutralModeValue = NeutralModeValue.Brake;

        motor = new TalonFX(CANIds.TALONFX_ID);
        TalonFXConfiguration configs = new TalonFXConfiguration();

        configs.MotorOutput.withInverted(TalonFXConstants.invertedValue)
                .withNeutralMode(neutralModeValue);
        configs.Slot0.kP = TalonFXConstants.P;
        configs.Slot0.kI = TalonFXConstants.I;
        configs.Slot0.kD = TalonFXConstants.D;
        configs.CurrentLimits.withSupplyCurrentLimit(TalonFXConstants.CURRENT_LIMIT);
        configs.CurrentLimits.withStatorCurrentLimit(TalonFXConstants.CURRENT_LIMIT);

        motor.getConfigurator().apply(configs);
    }

    public boolean withinHardLimits() {
        return !(CWLimitSwitch.get() || CCWLimitSwitch.get());
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

        if (!CCWLimitSwitch.get()) { // TODO Check if CCW is "negative" speed
            motor.set(-0.1);
        } else {
            motor.set(0);
            motor.setPosition(0);
            setpoint = TalonFXConstants.CCW_SOFT_LIMIT;
            atUpPosition = false;
            isZeroed = true;
        }
    }

    public void run() {
        if (!isZeroed) {
            return;
        }
        setpoint = HelperFunctions.clamp(setpoint, TalonFXConstants.CCW_SOFT_LIMIT, TalonFXConstants.CW_SOFT_LIMIT);
        motor.setControl(new PositionVoltage(setpoint));
        System.out.println("Setpoint: " + setpoint);

        // if (withinHardLimits()) { // TODO Make hard limits only lock 1 direction of movement
        //     motor.setControl(new PositionVoltage(setpoint));
        // } else {
        //     setpoint = currentPosition;
        //     motor.set(0);
        // }

    }

    public void togglePosition(){
        atUpPosition = !atUpPosition;
        if(atUpPosition) setpoint = TalonFXConstants.CCW_SOFT_LIMIT;
        else if(!atUpPosition) setpoint = TalonFXConstants.CW_SOFT_LIMIT;
    }

    public void runWithController(double controlX){
        if(CCWLimitSwitch.get())
        System.out.println("PRESSED");
        double speed = HelperFunctions.map(controlX, -1, 1, -0.2, 0.2);
        motor.set(speed);
    }
}