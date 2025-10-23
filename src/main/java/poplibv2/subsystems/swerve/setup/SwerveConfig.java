package poplibv2.subsystems.swerve.setup;

import edu.wpi.first.math.geometry.Translation2d;
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
     * Creates a new SwerveConfig that can be used to create a new Swerve Drivetrain
     * @param moduleConstants SwerveModuleConstants objects that describe the swerve module. Should be ordered as: [Top Left, Top Right, Bottom Left, Bottom Right]
     * @param CANBus What CANBus everything is on
     * @param driveMotorType The MotorVendor of all of your drive motors
     * @param rotMotorType The MotorVendor of all of your rotation motors
     * @param drivePidConfig The PID Config for your drive motors
     * @param rotPidConfig The PID Config for your rotation motors
     * @param driveCurrentLimit The current limit of your drive motors
     * @param rotCurrentLimit The current limit of your rotation motors
     * @param type The type of swerve module you are using
     * @param wheelDiameter The wheel diameter in inches, default is 4
     * @param gyroConfig The config used to create a Gyro
     * @param cameraConfigs The configs for creating Cameras
     * @param limelightConfigs The configs for creating Limelights
     * @param tuningEnabled Whether or not to enable tuning
     * @return The final SwerveConfig to be passed in when creating a new Swerve
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
        double wheelDiameter,
        Translation2d[] wheelPosForSDK,
        PigeonConfig gyroConfig, 
        CameraConfig[] cameraConfigs,
        LimelightConfig[] limelightConfigs,
        boolean tuningEnabled) {
        
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
            moduleConfigs[i] = new SwerveModuleConfig(driveMotorConfig, rotMotorConfig, canCoderConfig, i, type.maxSpeed, type.maxAngularVelocity);
        }
        return new SwerveConfig(moduleConfigs, gyroConfig, wheelDiameter, cameraConfigs, limelightConfigs, wheelPosForSDK, tuningEnabled);
    }

    public SwerveModuleConfig[] swerveModuleConfigs;
    public PigeonConfig gyro;
    public static Distance wheelCircumference;
    public CameraConfig[] cameraConfigs;
    public LimelightConfig[] limelightConfigs;
    public static boolean tuningEnable;
    public Translation2d[] wheelPos;

    /**
     * INTERNAL POPLIB FUNCTION.
     * 
     * USE THE generateConfig to create a new config
     * @param swerveModuleConfigs
     * @param gyro
     * @param wheelDiameter
     * @param cameraConfigs
     * @param limelightConfigs
     * @param wheelPos
     */
    private SwerveConfig(SwerveModuleConfig[] swerveModuleConfigs, 
        PigeonConfig gyro, double wheelDiameter, CameraConfig[] cameraConfigs, 
        LimelightConfig[] limelightConfigs, Translation2d[] wheelPos, boolean tuningEnabled) {
        wheelCircumference = Units.Inches.of(wheelDiameter).times(Math.PI);
        this.swerveModuleConfigs = swerveModuleConfigs;
        this.gyro = gyro;
        this.cameraConfigs = cameraConfigs;
        this.limelightConfigs = limelightConfigs;
        this.wheelPos = wheelPos;
        tuningEnable = tuningEnabled;
    }
}