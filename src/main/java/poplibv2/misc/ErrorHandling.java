package poplibv2.misc;

import com.revrobotics.REVLibError;

import edu.wpi.first.wpilibj.DriverStation;

public class ErrorHandling {
    /**
     * Handles a error that happens when a config fails to apply to a motor.
     * @param error The potienial error
     * @param useCase Why the error happened
     */
    public static void handleRevLibError(REVLibError error, String useCase) {
        if (error != REVLibError.kOk) {
            System.out.println("REVLibError: " + error.toString() + " when " + useCase);
        }
    } 

    /**
     * Complains about pid.
     */
    public static void complainAboutIncorrectUseOfPID(String m) {
        DriverStation.reportError("IncorrectUseOfPIDException: " + m, false);
    }
    

    /**
     * Complains about CAN.
     */
    public static void complainAboutDuplicatedCANIDException(String m) {
        DriverStation.reportError("DuplicatedCANIDException: " + m, false);
    }
}
