package POPLib.Subsytems.Elevator;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import POPLib.Control.FFConfig;
import POPLib.Math.MathUtil;
import POPLib.Motor.FollowerConfig;
import POPLib.Motor.MotorConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class SparkElevator extends Elevator {
    SparkMax leadMotor;
    SparkMax followMotor;
    boolean usePID;

    public SparkElevator(MotorConfig motorConfig, FollowerConfig followerConfig, FFConfig ffConfig, boolean tuningMode, boolean usePID, String subsystemName) {
        super(ffConfig, tuningMode, subsystemName);

        this.usePID = usePID;
        leadMotor = motorConfig.createSparkMax();
        leadMotor.getEncoder().setPosition(0.0);
        followMotor = followerConfig.createSparkMax();
        followMotor.getEncoder().setPosition(0.0);
    }

    @Override
    public void periodic() {
        super.tuning.updatePID(leadMotor);
        if (usePID) {
            leadMotor.getClosedLoopController().setReference(
                super.setpoint.get(), 
                ControlType.kPosition, 
                ClosedLoopSlot.kSlot1, 
                super.feedforward.calculate(getEncoderPos(), 0.0));
        }
        SmartDashboard.putNumber("Elevator lead motor pos", getEncoderPos());
        if (followMotor != null) {
            SmartDashboard.putNumber("Elevator follow motor pos", followMotor.getEncoder().getPosition());
        }
    }

    public double getEncoderPos() {
        return leadMotor.getEncoder().getPosition();
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
