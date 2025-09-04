package poplibv2.control_systems;

import poplibv2.misc.TunableNumber;
import poplibv2.motors.Motor;

public class PIDTuning {
    private final TunableNumber kP;
    private final TunableNumber kI;
    private final TunableNumber kD;
    private final TunableNumber kF;
    private final boolean tuningMode;

    /**
     * Creates a new PIDTuning helper
     * @param motorName The motor name that you are doing the PID Tuning for
     * @param pid The current PID Config
     * @param kTuningMode whether or not to do pid tuning
     */
    public PIDTuning(String motorName, PIDConfig pid, boolean tuningMode) {
        this(motorName, pid.P, pid.I, pid.D, pid.F, tuningMode);
    }

    private PIDTuning(String motorName, double motor_kP, double motor_kI, double motor_kD, double motor_kF, boolean kTuningMode){
        kP = new TunableNumber(motorName + "kP", motor_kP, kTuningMode);
        kI = new TunableNumber(motorName + "kI", motor_kI, kTuningMode);
        kD = new TunableNumber(motorName + "kD", motor_kD, kTuningMode);
        kF = new TunableNumber(motorName + "kF", motor_kF, kTuningMode);
        tuningMode = kTuningMode;
    }

    /**
     * POPLIB INTERNAL FUNCTION
     * <p></p>
     * @return if the pid constants have been changed or not.
     */
    private boolean isPIDChanged() {
        return (kP.hasChanged() || kI.hasChanged() || kD.hasChanged() || kF.hasChanged()) && (tuningMode);
    }

    /**
     * Updates the PID Constants on your motor using the constants on Smart Dashboard if tuning mode is on. 
     * @param motor
     */
    public void updatePID(Motor motor){
        if (isPIDChanged()) {
            PIDConfig config = new PIDConfig(kP.get(), kI.get(), kD.get(), kF.get());
            motor.changePID(config);
        }
    }
}