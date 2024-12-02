package POPLib.Swerve.SwerveConstants;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import POPLib.Motor.ConversionConfig;
import POPLib.Motor.MotorConfig;
import POPLib.Motor.MotorHelper;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants;

/**
 * Wrapper class for swerve module constants.
 * Angle motor encoders output in rotations 
 * Drive motors output in meters
 */
public class SwerveModuleConstants {
    public final int driveMotorId;
    public final int angleMotorId;
    public final int cancoderId;
    public final int moduleNumber;

    public final Rotation2d angleOffset;

    public final static NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
    public final static IdleMode angleIdleMode = IdleMode.kCoast;
    public static NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;
    public static IdleMode driveIdleMode = IdleMode.kBrake;

    public final static Distance wheelCircumference = Units.Inches.of(4).times(Math.PI);
    public final Distance driveDistancePerRotation;

    public final boolean swerveTuningMode;

    public final MotorConfig driveConfig;
    public final MotorConfig angleConfig;

    public final SDSModules moduleInfo;

    /**
     * Swerve Module Constants to be used when creating swerve modules.
     */
    public SwerveModuleConstants(int moduleNumber, int driveMotorId, int cancoderId, int angleMotorId, Rotation2d angleOffset, SDSModules moduleInfo,
            boolean swerveTuningMode, MotorConfig driveConfig, MotorConfig angleConfig) {
        this.moduleNumber = moduleNumber;
        this.angleOffset = angleOffset;

        this.driveMotorId = driveMotorId;
        this.cancoderId = cancoderId;
        this.angleMotorId = angleMotorId;

        this.moduleInfo = moduleInfo;

        driveDistancePerRotation = wheelCircumference.times(moduleInfo.driveGearRatio);

        this.swerveTuningMode = swerveTuningMode;

        this.driveConfig = driveConfig;
        this.driveConfig.conversion = new ConversionConfig(driveDistancePerRotation.in(Units.Meters), Units.Rotations);

        this.angleConfig = angleConfig;
        angleConfig.conversion = new ConversionConfig(moduleInfo.angleGearRatio, Units.Rotations);
    }

    public SwerveModuleConstants(int moduleNumber, Rotation2d angleOffset, SDSModules moduleInfo,
        boolean swerveTuningMode, MotorConfig driveConfig, MotorConfig angleConfig) {
            this(moduleNumber,  (moduleNumber * 3) + 1, (moduleNumber * 3) + 2, (moduleNumber * 3) + 3, angleOffset, moduleInfo, swerveTuningMode, driveConfig, angleConfig);
    }

    public static SwerveModuleConstants[] generateConstants(Rotation2d[] angleOffsets, SDSModules moduleInfo,
            boolean swerveTuningMode, MotorConfig driveConfig, MotorConfig angleConfig) {
        SwerveModuleConstants[] ret = new SwerveModuleConstants[4];

        for (int i = 0; i < 4; ++i) {
            ret[i] = new SwerveModuleConstants(
                i,
                angleOffsets[i],
                moduleInfo,
                swerveTuningMode,
                driveConfig,
                angleConfig
            );
        }

        return ret;
    }

    public static SwerveModuleConstants[] generateConstants(Rotation2d[] angleOffsets,Boolean[] flippedModules, Integer[] canIds, SDSModules moduleInfo,
            boolean swerveTuningMode, MotorConfig driveConfig, MotorConfig angleConfig) {
        SwerveModuleConstants[] ret = new SwerveModuleConstants[4];

        MotorConfig invertedDrive = driveConfig.getInvertedConfig();
        MotorConfig invertedAngle = angleConfig.getInvertedConfig();
        for (int i = 0; i < 4; ++i) {
            ret[i] = new SwerveModuleConstants(
                i,
                canIds[i*3],
                canIds[i*3+1],
                canIds[i*3+2],
                angleOffsets[i],
                moduleInfo,
                swerveTuningMode,
                flippedModules[i] ? invertedDrive : driveConfig,
                flippedModules[i] ? invertedAngle : angleConfig
            );
        }

        return ret;
    }

    public static SwerveModuleConstants[] generateConstants(Rotation2d[] angleOffsets, Integer[] canIds, SDSModules moduleInfo,
    boolean swerveTuningMode, MotorConfig driveConfig, MotorConfig angleConfig) {
        SwerveModuleConstants[] ret = new SwerveModuleConstants[4];

        for (int i = 0; i < 4; ++i) {
            ret[i] = new SwerveModuleConstants(
                i,
                canIds[i*3],
                canIds[i*3+1],
                canIds[i*3+2],
                angleOffsets[i],
                moduleInfo,
                swerveTuningMode,
                driveConfig,
                angleConfig
            );
        }

        return ret;
        }

    public SparkMax getDriveNeo() {
        return driveConfig.setConfig(new SparkMax(driveMotorId, MotorType.kBrushless));
    }

    public TalonFX getDriveFalcon() {
        return driveConfig.setConfig(new TalonFX(driveMotorId, Constants.Ports.CANIVORE_NAME));
    }

    public SparkMax getAngleNeo() {
        SparkMax neo = new SparkMax(angleMotorId, MotorType.kBrushless);
        SparkMaxConfig config = angleConfig.getSparkMaxConfig();
        config.closedLoop.positionWrappingEnabled(true);
        config.closedLoop.positionWrappingMinInput(0);
        config.closedLoop.positionWrappingMaxInput(360);

        MotorHelper.applySparkMaxConfig(config, neo, ResetMode.kResetSafeParameters);

        return neo;
    }

    public TalonFX getAngleFalcon() {
        return angleConfig.setConfig(new TalonFX(angleMotorId, Constants.Ports.CANIVORE_NAME));
    }

    public CANcoder getCanCoder() {
        CANcoder angleEncoder = new CANcoder(cancoderId, Constants.Ports.CANIVORE_NAME);

        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        config.MagnetSensor.MagnetOffset = -angleOffset.getRotations();

        angleEncoder.getConfigurator().apply(config);

        return angleEncoder;
    }
}
