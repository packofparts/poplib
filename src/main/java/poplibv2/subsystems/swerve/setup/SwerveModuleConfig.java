package poplibv2.subsystems.swerve.setup;

import edu.wpi.first.units.measure.LinearVelocity;
import poplibv2.motors.MotorConfig;
import poplibv2.sensors.absolute_encoder.CANCoderConfig;

public class SwerveModuleConfig {
    public MotorConfig driveMotorConfig;
    public MotorConfig rotMotorConfig;
    public CANCoderConfig canCoderConfig;
    public int id;
    public LinearVelocity maxSpeed;

    /**
     * POPLIB INTERNAL FUNCTION.
     * This is a container for storing configs for modules.
     * @param driveMotorConfig 
     * @param rotMotorConfig
     * @param canCoderConfig
     * @param id
     */
    public SwerveModuleConfig(MotorConfig driveMotorConfig, MotorConfig rotMotorConfig, CANCoderConfig canCoderConfig, int id, LinearVelocity maxSpeed) {
        this.driveMotorConfig = driveMotorConfig;
        this.rotMotorConfig = rotMotorConfig;
        this.canCoderConfig = canCoderConfig;
        this.id = id;
        this.maxSpeed = maxSpeed;
    }
    
}
