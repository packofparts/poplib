package POPLib.Subsytems.Pivot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import POPLib.Control.ArmFFConfig;
import POPLib.Motor.MotorConfig;
import POPLib.Motor.MotorHelper;
import POPLib.Sensors.AbsoluteEncoder.AbsoluteEncoderConfig;
import POPLib.SmartDashboard.PIDTuning;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SparkPivot extends Pivot {
    private final CANSparkMax leadMotor;
    private final CANSparkMax followerMotor;
    private final PIDTuning pid;

    public SparkPivot(MotorConfig leadConfig, MotorConfig followerConfig, double gearRatio, boolean invertedMotors, ArmFFConfig ffConfig, AbsoluteEncoderConfig absoluteConfig, boolean tuningMode, String subsytemName) {
        super(ffConfig, absoluteConfig, tuningMode, subsytemName);
        leadMotor = leadConfig.createSparkMax();
        followerMotor = followerConfig.createSparkMax();
        
        MotorHelper.setDegreeConversionFactor(leadMotor, gearRatio);
        MotorHelper.setDegreeConversionFactor(followerMotor,gearRatio);

        followerMotor.follow(leadMotor, invertedMotors);

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

        leadMotor.getPIDController().setReference(
            setpoint.get(), 
            ControlType.kPosition,
            0,
            ff.calculate(Math.toRadians(leadMotor.getEncoder().getPosition()), 0)
        );
    }

    @Override
    public void resetToAbsolutePosition() {
        leadMotor.getEncoder().setPosition(getAbsolutePosition());
    } 
}
