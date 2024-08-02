package POPLib.Subsytems.Flywheel;

import POPLib.SmartDashboard.TunableNumber;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Flywheel extends SubsystemBase {
    protected final TunableNumber setpoint;
 
    protected Flywheel(String subsytemName, boolean tuningMode) {
        super(subsytemName);
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
