package POPLib.Subsytems.Elevator;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import POPLib.Control.FFConfig;
import POPLib.Motor.FollowerConfig;
import POPLib.Motor.MotorConfig;

public class TalonElevator extends Elevator {
    TalonFX leadMotor;
    TalonFX followMotor;
    boolean usePID;
    PositionDutyCycle position;
    NeutralOut neutral;

    public TalonElevator(MotorConfig motorConfig, FollowerConfig followerConfig, FFConfig ffConfig, boolean tuningMode, boolean usePID, String subsystemName) {
        super(ffConfig, tuningMode, subsystemName);
    
        this.usePID = usePID;
        leadMotor = motorConfig.createTalon();
        neutral = lead
        leadMotor.setPosition(0.0);
        followMotor = followerConfig.createTalon();
        followMotor.setPosition(0.0);
    }

    @Override
    public void periodic() {
        super.tuning.updatePID(leadMotor);
        if (usePID) {
            leadMotor.setControl(super.setpoint.get())
        }
        SmartDashboard.putNumber("Elevator lead motor pos", getEncoderPos());
        if (followMotor != null) {
            SmartDashboard.putNumber("Elevator follow motor pos", followMotor.getEncoder().getPosition());
        }
    }
    
}
