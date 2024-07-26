package POPLib.Subsytems.Flywheel;

import com.ctre.phoenix6.signals.Led1OffColorValue;
import com.revrobotics.CANSparkBase.ControlType;
import POPLib.Control.PIDConfig;
import POPLib.Motor.MotorConfig;
import POPLib.SmartDashboard.PIDTuning;
import POPLib.SmartDashboard.TunableNumber;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Flywheel extends SubsystemBase {
    protected final String subsytemName;
    protected final TunableNumber setpoint;
 
    protected Flywheel(String subsytemName, boolean tuningMode) {
        this.subsytemName = subsytemName;
        this.setpoint = new TunableNumber(subsytemName + " flywheel setpoint", 0, tuningMode);
    } 

    public abstract double getError();
 
    public void updateSetpoint(double setpoint) {
        this.setpoint.setDefault(setpoint);
    }

    public Command updateSetpointCommand(double setpoint) {
        return runOnce(() -> this.setpoint.setDefault(setpoint));
    }

    public Command updateSetpointCommand(double setpoint, double maxError) {
        return run(() -> this.setpoint.setDefault(setpoint)).until(() -> getError() < maxError);
    }
}
