package poplibv2.motors;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import poplibv2.control_systems.PIDConfig;

/**
 * A container for all the Contansts a Motor needs. Also in charge of creating a motor object.
 */
public class MotorConfig {
    private int canID;
    private String canBUS;
    private MotorVendor motorVendor;
    private PIDConfig pidConfig;
    private int currentLimit;
    private boolean inverted;
    private IdleBehavior idleMode;
    private ConversionConfig conversionConfig;
    private boolean isConfiguredWithPID;
    private Motor motor = null; // used to mimic "singleton" behavior

    /**
     * The constuctor for creating the motorConfig
     * @param canID The can ID that is used to access the motor
     * @param canBUS What can bus the motor is on. Default is "rio"
     * @param motorVendor What type of motor this is
     * @param pidConfig The PID constants you want to apply to this motor
     * @param currentLimit The current limit that you want to set in software
     * @param inverted Whether or not the motor should be inverted
     * @param idleMode What the default behavior of the motor should be
     * @param conversionConfig What the gear box conversion constants are for this motor 
     */
    public MotorConfig(int canID, String canBUS, MotorVendor motorVendor, PIDConfig pidConfig, 
    int currentLimit, boolean inverted, IdleBehavior idleMode, ConversionConfig conversionConfig) {
        this.canID = canID;
        this.canBUS = canBUS;
        this.motorVendor = motorVendor;
        this.pidConfig = pidConfig;
        this.currentLimit = currentLimit;
        this.inverted = inverted;
        this.idleMode = idleMode;
        this.conversionConfig = conversionConfig;
    }

    /**
     * INTERNAL POPLIB FUNCTION.
     * <p></p>
     * Used to apply the current MotorConfig and fit it into a SparkMaxConfig
     * @return The SparkMaxConfig object
     */
    private SparkMaxConfig createSparkMaxConfig() {
        SparkMaxConfig config = new SparkMaxConfig();

        config.inverted(inverted);
        config.idleMode(idleMode == IdleBehavior.BRAKE ? IdleMode.kBrake : IdleMode.kCoast);
        config.smartCurrentLimit(currentLimit);
        isConfiguredWithPID = pidConfig.applyToMotor(config);
        conversionConfig.applyToMotor(config);

        return config;
    }

    /**
     * INTERNAL POPLIB FUNCTION.
     * <p></p>
     * Used to apply the current MotorConfig and fit it into a TalonFXConfiguration
     * @return The TalonFXConfiguration object
     */
    private TalonFXConfiguration createTalonFXConfiguration() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.StatorCurrentLimit = currentLimit;
        config.CurrentLimits.SupplyCurrentLimitEnable = false;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.MotorOutput.NeutralMode = idleMode == IdleBehavior.BRAKE ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        config.MotorOutput.Inverted = inverted ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
        isConfiguredWithPID = pidConfig.applyToMotor(config);
        conversionConfig.applyToMotor(config);
        return config;
    }

    /**
     * Creates a new Motor. If you have already created a motor using this MotorConfig, 
     * it returns the same Motor object. This is to stop your project from crashing by 
     * creating multiple Motors objects referencing the same motor.
     * @return
     */
    public Motor createMotor() {
        if (motor != null) {
            return motor;
        }
        if (motorVendor == MotorVendor.REV_ROBOTICS_SPARK_MAX) {
            SparkMaxConfig config = createSparkMaxConfig();
            motor = new Motor(config, canID, isConfiguredWithPID);
            return motor;
        } else {
            TalonFXConfiguration config = createTalonFXConfiguration();
            motor = new Motor(config, canID, canBUS, isConfiguredWithPID);
            return motor;
        }
    }
}
