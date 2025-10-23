package poplibv2.controllers.io;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * A joystick set up that uses two Microsoft flight simulator joysticks as the driver controller
 * and one XboxController as the operator controller. (we dont use this anymore.)
 */
public class Joysticks extends IO {
    public static Joysticks instance;

    private CommandJoystick driveJoystick;
    private CommandJoystick rotJoystick;
    private CommandXboxController operatorController;
    
    public static Joysticks getInstance() {
        if (instance == null) {
            instance = new Joysticks();
        }

        return instance;
    } 

    public static final class OIConstants  {
        public static final int DRIVE_PORT = 0;
        public static final int ROT_PORT = 1;
        public static final int OPERATOR_PORT = 2;
    }

    public Joysticks() {
        driveJoystick = new CommandJoystick(OIConstants.DRIVE_PORT);
        rotJoystick = new CommandJoystick(OIConstants.ROT_PORT);
        operatorController = new CommandXboxController(OIConstants.OPERATOR_PORT);
    }

    @Override
    public double getDriveTrainRotation() {
        return rotJoystick.getX();
    }

    @Override
    public double getDriveTrainTranslationY() {
        return driveJoystick.getY();
    }

    @Override
    public double getDriveTrainTranslationX() {
        return driveJoystick.getX();
        
    }

    @Override
    public CommandGenericHID getDriverController() {
        return driveJoystick;
    }

    @Override
    public CommandGenericHID getOperatorController() {
        return operatorController;
    }
}
