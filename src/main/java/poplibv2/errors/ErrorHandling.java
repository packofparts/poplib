package poplibv2.errors;

import com.revrobotics.REVLibError;

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
}
