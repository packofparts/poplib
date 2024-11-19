package POPLib.Subsytems.Flywheel;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import POPLib.Math.MathUtil;
import POPLib.Motor.MotorConfig;
import POPLib.SmartDashboard.PIDTuning;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SparkFlywheel extends Flywheel {
    CANSparkMax leadMotor; 
    CANSparkMax followerMotor; 
    PIDTuning leadPidTuning;
 
    protected SparkFlywheel(MotorConfig leadConfig, MotorConfig followerConfig, String subsytemName, boolean tuningMode, boolean motorsInverted) {
        super(subsytemName, tuningMode);

        this.leadMotor = leadConfig.createSparkMax();
        this.followerMotor = followerConfig.createSparkMax();

        followerMotor.follow(leadMotor, motorsInverted);
    } 

    public double getError(double setpoint) {
        return MathUtil.getError(leadMotor, setpoint);
    }

    public void log() {
        SmartDashboard.putNumber(getName() + " velocity ", leadMotor.getEncoder().getVelocity());
    }

    @Override
    public void periodic() {
        leadPidTuning.updatePID(leadMotor);
        leadMotor.getPIDController().setReference(setpoint.get(), ControlType.kVelocity);
    }
}
