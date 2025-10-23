package poplibv2.subsystems.pivot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import poplibv2.control_systems.PIDTuning;
import poplibv2.misc.TunableNumber;
import poplibv2.motors.Motor;

public class Pivot extends SubsystemBase {
    private TunableNumber setpoint;
    private PIDTuning tuning;
    private Motor motor;
    private double maxRotation;

    
}
