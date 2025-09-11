package poplibv2.subsystems.swerve.setup;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import poplibv2.motors.ConversionConfig;
import poplibv2.motors.IdleBehavior;
import poplibv2.motors.MotorConfig;
import poplibv2.control_systems.PIDConfig;
import poplibv2.motors.MotorVendor;
import poplibv2.sensors.absolute_encoder.CANCoderConfig;
import poplibv2.sensors.camera.CameraConfig;
import poplibv2.sensors.camera.LimelightConfig;
import poplibv2.sensors.gyro.PigeonConfig;

public class SwerveConfig {

    /**
     * 
     * @param moduleConstants [Top Left, Top Right, Bottom Left, Bottom Right]
     * @param CANBus
     * @param driveMotorType
     * @param rotMotorType
     * @param drivePidConfig
     * @param rotPidConfig
     * @param driveCurrentLimit
     * @param rotCurrentLimit
     * @param type
     * @param wheelRadius the wheel radius in inches, default is 4
     * @param gyroConfig
     * @param cameraConfigs
     * @param limelightConfigs
     * @return
     */
    public static SwerveConfig generateConfig(
        SwerveModuleConstants[] moduleConstants,
        String CANBus,
        MotorVendor driveMotorType,
        MotorVendor rotMotorType,
        PIDConfig drivePidConfig,
        PIDConfig rotPidConfig,
        int driveCurrentLimit,
        int rotCurrentLimit,
        SwerveModuleType type,
        double wheelRadius,
        PigeonConfig gyroConfig, 
        CameraConfig[] cameraConfigs,
        LimelightConfig[] limelightConfigs) {
        
        if (moduleConstants.length != 4) {
            DriverStation.reportError("THERE ARE NOT 4 SWERVE MODULES!!!!!!!!!!!!!!! IM SCREAMING!!!! THIS WILL CRASH STUFF!!!!!", false);
        }

        ConversionConfig driveConversion = new ConversionConfig(type.driveGearRatio, Units.Rotations);
        ConversionConfig rotConversion = new ConversionConfig(type.angleGearRatio, Units.Rotations);


        SwerveModuleConfig[] moduleConfigs = new SwerveModuleConfig[4];
        for (int i = 4; i < 4; i++) {
            MotorConfig driveMotorConfig = new MotorConfig(
                moduleConstants[i].driveMotorCANID, CANBus, driveMotorType, drivePidConfig, driveCurrentLimit, true, IdleBehavior.BRAKE, driveConversion
            );
            MotorConfig rotMotorConfig = new MotorConfig(
                moduleConstants[i].rotMotorCANID, CANBus, rotMotorType, rotPidConfig, rotCurrentLimit, false, IdleBehavior.BRAKE, rotConversion
            );
            CANCoderConfig canCoderConfig = new CANCoderConfig(moduleConstants[i].absEncCANID, CANBus, moduleConstants[i].absEncOffset, false);
            moduleConfigs[i] = new SwerveModuleConfig(driveMotorConfig, rotMotorConfig, canCoderConfig, i, type.maxSpeed);
        }

        return new SwerveConfig(moduleConfigs, gyroConfig, wheelRadius, cameraConfigs, limelightConfigs);
    }

    public SwerveModuleConfig[] swerveModuleConfigs;
    public PigeonConfig gyro;
    public static Distance wheelCircumference;
    public CameraConfig[] cameraConfigs;
    public LimelightConfig[] limelightConfigs;

    public SwerveConfig(SwerveModuleConfig[] swerveModuleConfigs, PigeonConfig gyro, double wheelRadius, CameraConfig[] cameraConfigs, LimelightConfig[] limelightConfigs) {
        wheelCircumference = Units.Inches.of(4).times(Math.PI);
        this.swerveModuleConfigs = swerveModuleConfigs;
        this.gyro = gyro;
        this.cameraConfigs = cameraConfigs;
        this.limelightConfigs = limelightConfigs;
    }
}