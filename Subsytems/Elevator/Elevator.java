package POPLib.Subsytems.Elevator;

import POPLib.Control.FFConfig;
import POPLib.Control.PIDConfig;
import POPLib.SmartDashboard.PIDTuning;
import POPLib.SmartDashboard.TunableNumber;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class Elevator extends SubsystemBase{
    protected final TunableNumber setpoint;
    protected PIDTuning tuning;
    protected ElevatorFeedforward feedforward;
    protected DigitalInput limitSwitch;

    public Elevator(FFConfig ffConfig, boolean tuningMode, String subsytemName) {
        super(subsytemName);

        setpoint = new TunableNumber("Elevator Setpoint", 0, tuningMode);
        tuning = new PIDTuning("Elevator", PIDConfig.getZeroPid(), tuningMode);
        feedforward = ffConfig.getElevatorFeedforward();
    }

    public abstract void periodic();

    public abstract double getError(double setpoint);

    // this is from the frc2025 repo, plz check that it works
    public Command moveElevator(double setPoint, double error) {
        return run(() -> setpoint.setDefault(setPoint)).
        until(() -> getError(setPoint) < error);
    }

    public abstract Command moveUp(double speed);

    public abstract Command moveDown(double speed);

    public abstract Command stop();
}