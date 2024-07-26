package POPLib.Subsytems.Pivot;

import POPLib.Control.ArmFFConfig;
import POPLib.Sensors.AbsoluteEncoder.AbsoluteEncoder;
import POPLib.Sensors.AbsoluteEncoder.AbsoluteEncoderConfig;
import POPLib.SmartDashboard.TunableNumber;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Pivot extends SubsystemBase {
    protected final AbsoluteEncoder absoluteEncoder;
    protected final ArmFeedforward ff;
    protected final TunableNumber setpoint;

    public Pivot(ArmFFConfig ffConfig, AbsoluteEncoderConfig absoluteConfig, boolean tuningMode, String subsytemName) {
        super(subsytemName);

        absoluteEncoder = absoluteConfig.generateAbsoluteEncoder();
        ff = ffConfig.getArmFeedforward();
        setpoint = new TunableNumber("Pivot Setpoint " + subsytemName, 0, tuningMode);
    }

    public Command moveWrist(double position, double error) {
        return run(() -> {
            setpoint.setDefault(position);
        }).until(() -> atSetpoint(error));
    }

    public abstract boolean atSetpoint(double error);

    public abstract void resetToAbsolutePosition();

    public double getAbsolutePosition() {
        return absoluteEncoder.getPosition();
    }

    public void log() {
        SmartDashboard.putNumber("Absoulte Position " + getName(), getAbsolutePosition()); 
    }
}
