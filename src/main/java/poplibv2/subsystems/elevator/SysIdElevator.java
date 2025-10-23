package poplibv2.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class SysIdElevator {
    SysIdRoutine routine;
    
    /**
     * Runs a System Identification on the elevator which can be used to find kF (kV) constants for drive motors
     * @param Elevator the Elevator Subsystem
     */
    public SysIdElevator(Elevator elevator) {
        routine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(elevator::runSysIdRoutine, elevator::sysIdLogMotors, elevator, "Elevator")
        );
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
    }

}
