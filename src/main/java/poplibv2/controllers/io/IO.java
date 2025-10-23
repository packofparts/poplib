package poplibv2.controllers.io;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** 
* Abstract class for OI control
*/
abstract public class IO {
    public static final double DEADBAND = 0.1;

    public IO() { }

    /**
     * Gets the speed that the driver wants the drive train to rotate
     * @return the speed from -1.0 to 1.0
     */
    public abstract double getDriveTrainRotation();

    /**
     * Gets the speed that the driver wants the drive train to move forward 
     * @return the forward speed from -1.0 to 1.0 
     */
    public abstract double getDriveTrainTranslationY();

    /**
     * Gets the speed that the driver wants the drive train to move right
     * @return the right speed from -1.0 to 1.0
     */
    public abstract double getDriveTrainTranslationX();

    /**
     * Gets the DriverController
     * @return a CommandGenericHID that can be used for button mapping
     */
    public abstract CommandGenericHID getDriverController();
    
    /**
     * Gets the OperatorController
     * @return a CommandGenericHID that can be used for button mapping
     */
    public abstract CommandGenericHID getOperatorController();

    /**
     * Gets the value of an axis on a controller.
     * @param id the axis id
     * @param controller the controller
     * @return the value from -1.0 to 1.0
     */
    protected double getRawAxis(int id, CommandGenericHID controller) {
        return controller.getHID().getRawAxis(id) * -1;
    }

    /**
     * Gets a Trigger that can be used to map a button to an command
     * @param id the button id
     * @return the Trigger
     */
    public Trigger getDriverButton(int id) {
        return getDriverController().button(id);
    }

    /**
     * Gets a Trigger that can be used to map a trigger(like the d-pad (the povs) or the left and right triggers) to an command
     * @param id the trigger id
     * @return the Trigger
     */
    public Trigger getDriverTrigger(int id) {
        return getDriverController().axisGreaterThan(id, 0.5);
    }

    /**
     * Gets a Trigger that can be used to map a button to an command
     * @param id the button id
     * @return the Trigger
     */
    public Trigger getOperatorButton(int id) {
        return getOperatorController().button(id);
    }
    
    /**
     * Gets a Trigger that can be used to map a trigger(like the d-pad (the povs) or the left and right triggers) to an command
     * @param id the trigger id
     * @return the Trigger
     */
    public Trigger getOperatorrigger(int id) {
        return getOperatorController().axisGreaterThan(id, 0.5);
    }
}
