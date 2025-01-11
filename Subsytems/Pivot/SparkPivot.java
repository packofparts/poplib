package POPLib.Subsytems.Pivot;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import POPLib.Control.ArmFFConfig;
import POPLib.Motor.FollowerConfig;
import POPLib.Motor.MotorConfig;
import POPLib.Sensors.AbsoluteEncoder.AbsoluteEncoderConfig;
import POPLib.SmartDashboard.PIDTuning;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SparkPivot extends Pivot {
    private final SparkMax leadMotor;
    @SuppressWarnings("unused")
    private final SparkMax followerMotor;
    private final PIDTuning pid;

    public SparkPivot(MotorConfig leadConfig, FollowerConfig followerConfig, double gearRatio, ArmFFConfig ffConfig, AbsoluteEncoderConfig absoluteConfig, boolean tuningMode, String subsytemName) {
        super(ffConfig, absoluteConfig, tuningMode, subsytemName);
        leadMotor = leadConfig.createSparkMax();
        followerMotor = followerConfig.createSparkMax(leadMotor);

        pid = leadConfig.genPIDTuning("Pivot Motor " + subsytemName, tuningMode);

        resetToAbsolutePosition();
    }

    public boolean atSetpoint(double error, double setpoint) {
        return getError(setpoint) < error;
    }

    public double getError(double setpoint) {
        return Math.abs(leadMotor.getEncoder().getPosition() - setpoint);
    }

    @Override
    public void log() {
        super.log();
        SmartDashboard.putNumber("Lead Position " + getName(), leadMotor.getEncoder().getPosition());
    }

    @Override
    public void periodic() {
        pid.updatePID(leadMotor);

        // TODO: Move to mutable units
        leadMotor.getClosedLoopController().setReference(
            setpoint.get(), 
            ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            ff.calculate(leadMotor.getEncoder().getPosition(), 0)
        );
    }

    @Override
    public void resetToAbsolutePosition() {
        leadMotor.getEncoder().setPosition(getAbsolutePosition());
    } 
}
