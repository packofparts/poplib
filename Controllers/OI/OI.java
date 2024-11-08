package POPLib.Controllers.OI;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** 
 * Handels xbox controllers.
*/
abstract public class OI {
    public static final double DEADBAND = 0.1;

    public OI() {
    }


    public abstract double getDriveTrainRotation();

    public abstract double getDriveTrainTranslationY();

    public abstract double getDriveTrainTranslationX();

    // protected double getRawAxis(int id) {
    //     return getDriverController().getHID().getRawAxis(id) * -1;
    // }

    // // public abstract CommandGenericHID getDriverController();

    // public Trigger getDriverButton(int id) {
    //     return getDriverController().button(id);
    // }

    // // public abstract CommandGenericHID getOperatorController();

    // public Trigger getOperatorButton(int id) {
    //     return getOperatorController().button(id);
    // }
}
