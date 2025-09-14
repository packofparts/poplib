package poplibv2.subsystems.swerve;

import java.util.Arrays;
import java.util.Collections;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import poplibv2.sensors.camera.DetectedObject;
import poplibv2.control_systems.PIDConfig;
import poplibv2.control_systems.PIDTuning;
import poplibv2.sensors.camera.Camera;
import poplibv2.sensors.camera.Limelight;
import poplibv2.sensors.gyro.Pigeon;
import poplibv2.subsystems.swerve.setup.SwerveConfig;

public class Swerve extends SubsystemBase {
    
    private SwerveModule[] wheels;
    private Pigeon gyro;
    public Camera[] cameras;
    public Limelight[] limelights;
    public Field2d field;
    public Pose2d prevPose;
    public double prevPoseTimeStamp;
    public LinearVelocity maxSpeed;
    public AngularVelocity maxAngularVelocity;
    private PIDTuning driveMotorTuning;
    private PIDTuning rotMotorTuning;
    private SwerveDriveKinematics kinematics;
    private SwerveDrivePoseEstimator odometry;

    /**
     * Creates a new Swerve Drivetrain, and zeros all of the wheels.
     * Sets up all of the swerve things.
     * @param config
     */
    public Swerve(SwerveConfig config) {
        wheels = new SwerveModule[4];
        for (int i = 0; i < 4; i++) {
            wheels[i] = new SwerveModule(config.swerveModuleConfigs[i]);
        }

        gyro = new Pigeon(config.gyro);
        gyro.zeroGyro();

        cameras = new Camera[config.cameraConfigs.length];
        for (int i = 0; i < config.cameraConfigs.length; i++) {
            cameras[i] = new Camera(config.cameraConfigs[i]);
        }

        limelights = new Limelight[config.limelightConfigs.length];
        for (int i = 0; i < config.limelightConfigs.length; i++) {
            limelights[i] = new Limelight(config.limelightConfigs[i]);
        }

        field = new Field2d();
        driveMotorTuning = new PIDTuning("Swerve Motor Drive", new PIDConfig(), SwerveConfig.tuningEnable);
        rotMotorTuning = new PIDTuning("Swerve Motor Rotation", new PIDConfig(), SwerveConfig.tuningEnable);
        maxSpeed = config.swerveModuleConfigs[0].maxSpeed;
        maxAngularVelocity = config.swerveModuleConfigs[0].maxAngularVelocity;

        kinematics = new SwerveDriveKinematics(config.wheelPos);
        odometry = new SwerveDrivePoseEstimator(kinematics, gyro.getNormalizedRotation2dAngle(), getModulePositions(), new Pose2d(0.0, 0.0, this.gyro.getNormalizedRotation2dAngle()));

        updatePrevPose();
    }

    @Override
    public void periodic() {
        updatePID();
        updatePrevPose();
        updateOdom();
        field.setRobotPose(getRobotPose());
        log();
    }

    /**
     * Updates the PID Constants of each of the swerve module motors
     */
    private void updatePID() {
        for (int i = 0; i < 4; i++) {
            wheels[i].updatePID(driveMotorTuning, rotMotorTuning);
        }
    }

    /**
     * LOG EVERYTHING. (this will come in handy later so we can blame electrical).
     */
    private void log() {
        SmartDashboard.putNumber("Angle", MathUtil.inputModulus(gyro.getNormalizedAngle().in(Units.Degrees), 0, 360));
        SmartDashboard.putNumber("Robot Angle Velo", getAngleVelo());
        SmartDashboard.putNumber("Robot Velo", getDriveVelo());
        SmartDashboard.putData("Field2d", field);
        for (int i = 0; i < 4; i++) {
            wheels[i].log();
        }
    }

    /**
     * Updates the odometry using encoder and vision based methods.
     */
    private void updateOdom() {
        odometry.update(gyro.getNormalizedRotation2dAngle(), getModulePositions());
        for (int i = 0; i < cameras.length; i++) {
            Optional<EstimatedRobotPose> estimatedPose = cameras[i].getEstimatedPose(getRobotPose());
            if (estimatedPose.isPresent()) {
                odometry.addVisionMeasurement(estimatedPose.get().estimatedPose.toPose2d(), 
                                              estimatedPose.get().timestampSeconds, 
                                              cameras[i].getVisionStdDevs());
            }
        }
    }

    /**
     * Drives the robot oriented and applies desaturation
     * @param states
     */
    private void driveRobotOriented(SwerveModuleState[] states) {
        desaturateWheelSpeeds(states, maxSpeed.in(Units.MetersPerSecond));
        for (int i = 0; i < 4; i++) {
            wheels[i].setDesiredState(states[i]);
        }
    }

    /**
     * Drives the robot (as robot oriented)
     * @param vector (The forward/back and left/right speeds to drive in meters per second)
     * @param rot The speed to turn in radians per second
     */
    public void driveRobotOriented(Translation2d vector, double rot) {
        SwerveModuleState[] states = this.kinematics.toSwerveModuleStates(new ChassisSpeeds(vector.getX(), vector.getY(), rot));
        this.driveRobotOriented(states);
    }
  
    /**
     * Drives the robot (as robot oriented)
     * @param chassisSpeeds The speeds to follow
     */
    public void driveRobotOriented(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] states = this.kinematics.toSwerveModuleStates(chassisSpeeds);
        this.driveRobotOriented(states);
    }
    
    // Vector is in mps, and rot is in radians per sec
    // Also this is field oriented
    /**
     * Drives the robot as field oriented
     * @param vector (The forward/back and left/right speeds to drive as a percent of the drivebases max speed (a number from -1.0 to 1.0))
     * @param rot The rotational speed to drive as a percent of the drivebases max rotational speed (a number from -1.0 to 1.0)
     * @param color I swear I'm not racist it matters
     */
    public void drive(Translation2d vector, double rot, Alliance color) {
        vector = vector.times(maxSpeed.in(Units.MetersPerSecond));
        rot *= maxAngularVelocity.in(Units.RadiansPerSecond);

        vector = vector.rotateBy(new Rotation2d(
            Units.Degrees.of(color == Alliance.Red ? 180 : 0).minus(gyro.getLatencyCompensatedAngle())
        ));

        driveRobotOriented(vector, rot);
    }

    /**
     * Caps the wheel speeds once they are over the maximum wheel speed that the motors can handle
     * @param states
     * @param maxSpeed
     */
    public void desaturateWheelSpeeds(SwerveModuleState[] states, double maxSpeed) {
        double realMaxSpeed = Collections.max(Arrays.asList(states)).speedMetersPerSecond;

        if (realMaxSpeed > maxSpeed) {
            for (SwerveModuleState moduleState : states) {
                moduleState.speedMetersPerSecond =
                    (moduleState.speedMetersPerSecond / realMaxSpeed) * maxSpeed;
            }
        }
    }

    /**
     * Gets an array of the wheel positions
     * @return An array of SwerveModulePositions
     */
    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            positions[i] = wheels[i].getPose();
        }
        return positions;
    }

    /**
     * Gets the current robot estimated pose
     * @return the pose
     */
    public Pose2d getRobotPose() {
        return odometry.getEstimatedPosition();
    }

    /**
     * Updates the previous pose, used for drive and angle velocity calculations
     */
    private void updatePrevPose() {
        prevPose = odometry.getEstimatedPosition();
        prevPoseTimeStamp = System.currentTimeMillis();
    }

    /**
     * Rezeros all wheels to thier absolute encoders
     */
    public void rezeroAllWheels() {
        for (int i = 0; i < 4; i++) {
            wheels[i].resetToAbsolute();
        }
    }
    
    /**
     * Runs the drive motors at a specific voltage
     * @param voltage
     */
    public void runSysIdRoutine(Voltage voltage) {
        for (int i = 0; i < 4; i++) {
            wheels[i].runSysIdRoutine(voltage.in(Units.Volts));
        }
    }

    /**
     * Gets logs from the motors
     * @param log
     */
    public void sysIdLogMotors(SysIdRoutineLog log) {
        for (int i = 0; i < 4; i++) {
            wheels[i].logSysId(log);
        }
    }

    /**
     * Zeros the gyro.
     */
    public void resetGyro() {
        gyro.zeroGyro();
    }

    /**
     * A command to zero the gyro
     * @return
     */
    public Command resetGyroCommand() {
        return runOnce(() -> {
            resetGyro();
        });
    }

    /**
     * Gets the angluar velocity of the robot in radians per millisecond
     * @return
     */
    public double getAngleVelo() {
        return 1000 * (getRobotPose().getRotation().getRadians() - prevPose.getRotation().getRadians())
                / (System.currentTimeMillis() - prevPoseTimeStamp); // in radians per milisecond
    }

    /**
     * Gets the velocity of the robot in meters per millisecond
     * @return
     */
    public double getDriveVelo() {
        return 1000 * (getRobotPose().getTranslation().getNorm() - prevPose.getTranslation().getNorm())
                / (System.currentTimeMillis() - prevPoseTimeStamp); // in meters per milisecond
    }

    /**
     * Based on whether an object is detected using a limelight, adjusts the driver input to move closer to the object 
     * @param driverInput
     * @return the new driver input
     */
    public Transform2d addVisionMovementAdjustment(Transform2d driverInput) {
        DetectedObject bestDetection = null;
        double bestArea = -1.0;
        for (Limelight limelight : limelights) {
            Optional<DetectedObject> detection = limelight.getLastestDetection();
            if (detection.isPresent() && ((DetectedObject)detection.get()).area > bestArea) {
                bestDetection = (DetectedObject)detection.get();
                bestArea = bestDetection.area;
            }
        }

        if (bestArea != -1.0 && bestDetection != null) {
            Rotation2d newAngle = driverInput.getRotation().plus(Rotation2d.fromDegrees(bestDetection.xAngleOffset / 10.0));
            double newY = driverInput.getY() + bestDetection.xAngleOffset / 52.0;
            double newX = driverInput.getX() + 1.0 / bestDetection.area;
            return new Transform2d(newX, newY, newAngle);
        } else {
            return driverInput;
        }
    }

}
