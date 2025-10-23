package poplibv2.subsystems.swerve.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import poplibv2.subsystems.swerve.Swerve;

public class SysIdSwerve {
    SysIdRoutine routine;
    
    /**
     * Runs a System Identification on swerve which can be used to find kF (kV) constants for drive motors
     * @param swerve the Swerve Subsystem
     */
    public SysIdSwerve(Swerve swerve) {
        routine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(swerve::runSysIdRoutine, swerve::sysIdLogMotors, swerve, "Swerve")
        );
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
    }
}
