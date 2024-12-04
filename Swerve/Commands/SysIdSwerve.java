package POPLib.Swerve.Commands;

import POPLib.Swerve.SwerveTemplates.BaseSwerve;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class SysIdSwerve {
   SysIdRoutine routine;
   
   public SysIdSwerve(BaseSwerve swerve) {
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
