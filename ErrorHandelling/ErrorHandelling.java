package POPLib.ErrorHandelling;

import com.revrobotics.REVLibError;

public class ErrorHandelling {
    public static void handlRevLibError(REVLibError error, String useCase) {
        if (error != REVLibError.kOk) {
            System.out.println("REVLibError: " + error.toString() + " when " + useCase);
        }
    } 
}
