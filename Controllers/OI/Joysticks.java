package POPLib.Controllers.OI;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.simulation.JoystickSim;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class Joysticks extends OI {

    public static Joysticks instance;

    private CommandJoystick driveJoystick;
    private CommandJoystick transJoystick;
    private CommandXboxController operatorController;
    

    public static Joysticks getInstance() {
        if (instance == null) {
            instance = new Joysticks();
        }

        return instance;
    } 

    public static final class OIConstants  {
        public static final int DRIVE_PORT = 0;
        public static final int OPERATOR_PORT = 2;
    }

    private void JoystickOI() {
        driveJoystick = new CommandJoystick(OIConstants.DRIVE_PORT);
        transJoystick = new CommandJoystick(1);
        operatorController = new CommandXboxController(OIConstants.OPERATOR_PORT);
    }

    @Override
    public double getDriveTrainRotation() {
        return transJoystick.getRawAxis(0);
    }

    @Override
    public double getDriveTrainTranslationY() {
        return driveJoystick.getY();
    }

    @Override
    public double getDriveTrainTranslationX() {
        return driveJoystick.getX();
        
    }

    public CommandGenericHID getDriverController() {
        return driveJoystick;
    }

    public CommandGenericHID getOperatorController() {
        return operatorController;
    }

    protected double getRawAxis(int id) {
        return getDriverController().getHID().getRawAxis(id) * -1;
    }

    // public abstract CommandGenericHID getDriverController();

    public Trigger getDriverButton(int id) {
        return getDriverController().button(id);
    }

    // public abstract CommandGenericHID getOperatorController();

    public Trigger getOperatorButton(int id) {
        return getOperatorController().button(id);
    }
}
