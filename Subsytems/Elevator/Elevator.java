package POPLib.Subsytems.Elevator;

import POPLib.Control.FFConfig;
import POPLib.Control.PIDConfig;
import POPLib.SmartDashboard.PIDTuning;
import POPLib.SmartDashboard.TunableNumber;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase{
    TunableNumber setpoint;
    PIDTuning tuning;
    ElevatorFeedforward feedforward;

    public Elevator(FFConfig FFconfig, boolean tuningMode, String subsytemName) {
        super(subsytemName);

        setpoint = new TunableNumber("Elevator Setpoint", 0, tuningMode);
        tuning = new PIDTuning("Elevator", PIDConfig.getZeroPid(), tuningMode);
    }
}
