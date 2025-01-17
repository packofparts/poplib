package POPLib.Subsytems.Elevator;

import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import POPLib.Control.FFConfig;
import POPLib.Math.MathUtil;
import POPLib.Motor.FollowerConfig;
import POPLib.Motor.MotorConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class TalonElevator extends Elevator {
    TalonFX leadMotor;
    TalonFX followMotor;
    boolean usePID;
    PositionDutyCycle position;

    public TalonElevator(MotorConfig motorConfig, FollowerConfig followerConfig, FFConfig ffConfig, boolean tuningMode, boolean usePID, String subsystemName) {
        super(ffConfig, tuningMode, subsystemName);
    
        this.usePID = usePID;

        leadMotor = motorConfig.createTalon();
        position = new PositionDutyCycle(0.0).
        withSlot(leadMotor.getClosedLoopSlot().getValue());

        leadMotor.setPosition(0.0);
        followMotor = followerConfig.createTalon();
        followMotor.setPosition(0.0);
    }

    @Override
    public void periodic() {
        if (usePID) {
            super.tuning.updatePID(leadMotor);
            leadMotor.setControl(position.withPosition(super.setpoint.get()).withFeedForward(super.feedforward.getKg()));
        }
        SmartDashboard.putNumber("Elevator lead motor pos", leadMotor.getPosition().getValueAsDouble());
        if (followMotor != null) {
            SmartDashboard.putNumber("Elevator follow motor pos", followMotor.getPosition().getValueAsDouble());
        }
    }

    public double getError(double setpoint) {
        return MathUtil.getError(leadMotor, setpoint);
    }

    public Command moveUp(double speed) {
        return runOnce(() -> {
            leadMotor.set(Math.abs(speed));
        });
    }

    public Command moveDown(double speed) {
        return runOnce(() -> {
            leadMotor.set(-Math.abs(speed));
        });
    }

    public Command stop() {
        return runOnce(() -> {
            leadMotor.set(0.0);
        });
    }    
}
