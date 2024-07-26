package POPLib.Subsytems.Flywheel;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import POPLib.Control.PIDConfig;
import POPLib.Math.MathUtil;
import POPLib.Motor.MotorConfig;
import POPLib.SmartDashboard.PIDTuning;
import POPLib.SmartDashboard.TunableNumber;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class SparkFlywheel extends Flywheel {
    CANSparkMax leadMotor; 
    CANSparkMax followerMotor; 
    String subsytemName;
    TunableNumber setpoint;
    PIDTuning leadPidTuning;
 
    protected SparkFlywheel(MotorConfig leadConfig, MotorConfig followerConfig, String subsytemName, boolean tuningMode, boolean motorsInverted) {
        super(subsytemName, tuningMode);

        this.leadMotor = leadConfig.createSparkMax();
        this.followerMotor = followerConfig.createSparkMax();

        this.leadPidTuning = new PIDTuning(subsytemName + " flywheel", PIDConfig.getZeroPid(), tuningMode);

        followerMotor.follow(leadMotor, motorsInverted);

        this.setpoint = new TunableNumber(subsytemName + " flywheel setpoint", 0, tuningMode);
    } 

    public double getError() {
        return MathUtil.getError(leadMotor, setpoint);
    }

    public void updateSetpoint(double setpoint) {
        this.setpoint.setDefault(setpoint);
    }

    public Command updateSetpointCommand(double setpoint) {
        return runOnce(() -> this.setpoint.setDefault(setpoint));
    }

    public Command updateSetpointCommand(double setpoint, double maxError) {
        return run(() -> this.setpoint.setDefault(setpoint)).until(() -> getError() < maxError);
    }

    public void log() {
        SmartDashboard.putNumber(subsytemName + " velocity ", leadMotor.getEncoder().getVelocity());
    }

    @Override
    public void periodic() {
        leadPidTuning.updatePID(leadMotor);
        leadMotor.getPIDController().setReference(setpoint.get(), ControlType.kVelocity);
    }
}
