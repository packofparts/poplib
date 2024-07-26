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
        
        MotorHelper.setConversionFactor(leadMotor, gearRatio);
        MotorHelper.setConversionFactor(followerMotor,gearRatio);

        leadMotor.follow(followerMotor, invertedMotors);

        pid = leadConfig.genPIDTuning("Pivot Motor " + subsytemName, tuningMode);

        resetToAbsolutePosition();;
    }

    public boolean atSetpoint(double error) {
        return getError() < error;
    }

    public double getError() {
        return Math.abs(followerMotor.getEncoder().getPosition() - setpoint.get());
    }

    @Override
    public void log() {
        super.log();
        SmartDashboard.putNumber("Lead Position " + getName(), followerMotor.getEncoder().getPosition());

    }

    @Override
    public void periodic() {
        pid.updatePID(followerMotor);

        followerMotor.getPIDController().setReference(
            setpoint.get(), 
            ControlType.kPosition,
            0,
            ff.calculate(Math.toRadians(followerMotor.getEncoder().getPosition()), 0)
        );
    }

    @Override
    public void resetToAbsolutePosition() {
        leadMotor.getEncoder().setPosition(getAbsolutePosition());
    } 
}
