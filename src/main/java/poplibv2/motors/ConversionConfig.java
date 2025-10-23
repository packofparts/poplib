package poplibv2.motors;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Units;

/** 
 * Responsible for taking into account a gear ratio when trying to run a motor at a certain speed 
 * or turn a motor a certain amount of degrees
 */
public class ConversionConfig {
    public double gearRatio;
    public AngleUnit unit; 

    /**
     * The constructor for creating a ConversionConfig
     * @param gearRatio The ratio between the amount of rotation done on the mechinaism vs the amount the motor has turned
     * If you don't know, ask the mechanical or design team.
     * @param unit The unit the gear ratio is in
     */
    public ConversionConfig(double gearRatio, AngleUnit unit) {
        this.gearRatio = gearRatio;
        this.unit = unit;
    }

    /**
     * The default, basic gear ratio. Only use if there is no gear conversion happening.
     */
    public ConversionConfig() {
        this(1.0, Units.Rotations);
    }

    /**
     * POPLIB INTERNAL FUNCTION.
     * <p></p>
     * Updates the motor config for a Spark Max motor to include the gear ratio for both position and velocity.
     * @param config the SparkMax Motor config that creates is used to create the motor later on.
     * NOTE: this is NOT the poplib MotorConfig class.
     */
    public void applyToMotor(SparkMaxConfig config) {
        config.encoder.positionConversionFactor(unit.convertFrom(1.0, Units.Rotation) / gearRatio);
        config.encoder.velocityConversionFactor((unit.convertFrom(1.0, Units.Rotation) / gearRatio) / 60);
    }

    /**
     * POPLIB INTERNAL FUNCTION.
     * <p></p>
     * Updates the motor config for a Spark Max motor to include the gear ratio for both position and velocity.
     * @param config the Talon Motor config that creates is used to create the motor later on.
     * NOTE: this is NOT the poplib MotorConfig class.
     */
    public void applyToMotor(TalonFXConfiguration config) {
        config.Feedback.SensorToMechanismRatio =  1.0 / (unit.convertFrom(1.0, Units.Rotation) / gearRatio);
    }
}