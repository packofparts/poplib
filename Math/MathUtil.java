package POPLib.Math;

import com.revrobotics.CANSparkMax;

import POPLib.Sensors.AbsoluteEncoder.AbsoluteEncoder;
import POPLib.SmartDashboard.TunableNumber;


public class MathUtil {
   public static double clamp(double max, double min, double currVal) {
        if (currVal > max) {
            return max;
        } else if (currVal < min) {
            return min;
        }

        return currVal;
   } 

   public static double clamp(double max, double min, TunableNumber currVal) {
    return clamp(max, min, currVal.get());
   } 

   public static double getError(double target, double setpoint) {
    return Math.abs(target - setpoint);
   }

   public static double getError(CANSparkMax target, double setpoint) {
    return getError(target.getEncoder().getPosition(), setpoint);
   }

   public static double getError(CANSparkMax target, TunableNumber setpoint) {
    return getError(target.getEncoder().getPosition(), setpoint.get());
   }

   public static double getError(CANSparkMax target, AbsoluteEncoder setpoint) {
    return getError(target.getEncoder().getPosition(), setpoint.getNormalizedPosition());
   }
}
