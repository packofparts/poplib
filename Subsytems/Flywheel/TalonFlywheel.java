package POPLib.Subsytems.Flywheel;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import POPLib.Control.PIDConfig;
import POPLib.Math.MathUtil;
import POPLib.Motor.MotorConfig;
import POPLib.SmartDashboard.PIDTuning;
import POPLib.SmartDashboard.TunableNumber;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TalonFlywheel extends Flywheel {
    public TalonFX leadMotor; 
    public TalonFX followerMotor; 
 
    public PIDTuning leadPidTuning;

    VelocityDutyCycle velocity;
 
    protected TalonFlywheel(MotorConfig leadConfig, MotorConfig followerConfig, String subsytemName, boolean tuningMode, boolean motorsInverted) {
        super(subsytemName, tuningMode);

        this.leadMotor = leadConfig.createTalon();
        this.followerMotor = followerConfig.createTalon();

        this.leadPidTuning = new PIDTuning(subsytemName + " flywheel", PIDConfig.getZeroPid(), tuningMode);

        this.velocity = new VelocityDutyCycle(0.0);

        followerMotor.setControl(new Follower(leadConfig.canId, motorsInverted));
    } 

    public double getError(double setpoint) {
        return Math.abs(leadMotor.getVelocity().getValueAsDouble() - setpoint);
    }

    public void log() {
        SmartDashboard.putNumber(getName() + " velocity ", leadMotor.getVelocity().getValueAsDouble());
    }

    public double getVelocity() {
        return leadMotor.getVelocity().getValueAsDouble();
    }

    @Override
    public void periodic() {
        leadPidTuning.updatePID(leadMotor);

        if (setpoint.hasChanged()) {
            leadMotor.setControl(velocity.withVelocity(setpoint.get()));
        }
    } 
}
