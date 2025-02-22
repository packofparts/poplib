package poplib.smart_dashboard;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import poplib.control.PIDConfig;
import poplib.error_handling.ErrorHandling;

public class PIDTuning {
    private final TunableNumber kP;
    private final TunableNumber kI;
    private final TunableNumber kD;
    private final TunableNumber kF;
    private final boolean tuningMode;

    public PIDTuning(String motorName, PIDConfig pid, boolean kTuningMode) {
        this(motorName, pid.P, pid.I, pid.D, pid.F, kTuningMode);
    }

    public PIDTuning(String motorName, double motor_kP, double motor_kI, double motor_kD, double motor_kF, boolean kTuningMode){
        kP = new TunableNumber(motorName + "kP", motor_kP, kTuningMode);
        kI = new TunableNumber(motorName + "kI", motor_kI, kTuningMode);
        kD = new TunableNumber(motorName + "kD", motor_kD, kTuningMode);
        kF = new TunableNumber(motorName + "kF", motor_kF, kTuningMode);
        tuningMode = kTuningMode;
    }

    private boolean isPIDChanged() {
        return (kP.hasChanged() || kI.hasChanged() || kD.hasChanged() || kF.hasChanged()) && (tuningMode);
    }

    public void updatePID(SparkMax motor){
        if (isPIDChanged()) { 
            SparkMaxConfig config = new SparkMaxConfig();
            config.closedLoop.pidf(kP.get(),kI.get(), kD.get(), kF.get());
            
            ErrorHandling.handlRevLibError(
                motor.configure(config,  ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters),
                "updating PID for motor id " + motor.getDeviceId()
            );
        }
    }

    public void updatePID(TalonFX motor){
        if (isPIDChanged()) {
            Slot0Configs slot0Configs = new Slot0Configs();
            slot0Configs.kV = kF.get();
            slot0Configs.kP = kP.get();
            slot0Configs.kI = kI.get();
            slot0Configs.kD = kD.get();
            motor.getConfigurator().apply(slot0Configs);
        }
    }
}
