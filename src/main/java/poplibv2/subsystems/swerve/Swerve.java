package poplibv2.subsystems.swerve;

import java.util.Arrays;
import java.util.Collections;
import java.util.Iterator;
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
    private LinearVelocity maxSpeed;
    private AngularVelocity maxAngularVelocity;
    private PIDTuning driveMotorTuning;
    private PIDTuning rotMotorTuning;
    private SwerveDriveKinematics kinematics;
    private SwerveDrivePoseEstimator odometry;

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

    private void updatePID() {
        for (int i = 0; i < 4; i++) {
            wheels[i].updatePID(driveMotorTuning, rotMotorTuning);
        }
    }

    private void log() {
        SmartDashboard.putNumber("Angle", MathUtil.inputModulus(gyro.getNormalizedAngle().in(Units.Degrees), 0, 360));
        SmartDashboard.putNumber("Robot Angle Velo", getAngleVelo());
        SmartDashboard.putNumber("Robot Velo", getDriveVelo());
        for (int i = 0; i < 4; i++) {
            wheels[i].log();
        }
    }

    private void updateOdom() {
        for (int i = 0; i < cameras.length; i++) {
            Optional<EstimatedRobotPose> estimatedPose = cameras[i].getEstimatedPose(getRobotPose());
            if (estimatedPose.isPresent()) {
                odometry.addVisionMeasurement(estimatedPose.get().estimatedPose.toPose2d(), 
                                              estimatedPose.get().timestampSeconds, 
                                              cameras[i].getVisionStdDevs());
            }
        }
        odometry.update(gyro.getNormalizedRotation2dAngle(), getModulePositions());
    }

    private void driveRobotOriented(SwerveModuleState[] states) {
        desaturateWheelSpeeds(states, maxSpeed.in(Units.MetersPerSecond));
        for (int i = 0; i < 4; i++) {
            wheels[i].setDesiredState(states[i]);
        }
    }

    public void driveRobotOriented(Translation2d vector, double rot) {
        SwerveModuleState[] states = this.kinematics.toSwerveModuleStates(new ChassisSpeeds(vector.getX(), vector.getY(), rot));
        this.driveRobotOriented(states);
    }
  
    public void driveRobotOriented(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] states = this.kinematics.toSwerveModuleStates(chassisSpeeds);
        this.driveRobotOriented(states);
    }
    
    // Vector is in mps, and rot is in radians per sec
    // Also this is field oriented
    public void drive(Translation2d vector, double rot, Alliance color) {
        vector = vector.times(maxSpeed.in(Units.MetersPerSecond));
        rot *= maxAngularVelocity.in(Units.RadiansPerSecond);

        vector = vector.rotateBy(new Rotation2d(
            Units.Degrees.of(color == Alliance.Red ? 180 : 0).minus(gyro.getLatencyCompensatedAngle())
        ));

        driveRobotOriented(vector, rot);
    }

    public void desaturateWheelSpeeds(SwerveModuleState[] states, double maxSpeed) {
        double realMaxSpeed = Collections.max(Arrays.asList(states)).speedMetersPerSecond;

        if (realMaxSpeed > maxSpeed) {
            for (SwerveModuleState moduleState : states) {
                moduleState.speedMetersPerSecond =
                    (moduleState.speedMetersPerSecond / realMaxSpeed) * maxSpeed;
            }
        }
    }

    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            positions[i] = wheels[i].getPose();
        }
        return positions;
    }

    public Pose2d getRobotPose() {
        return odometry.getEstimatedPosition();
    }

    private void updatePrevPose() {
        prevPose = odometry.getEstimatedPosition();
        prevPoseTimeStamp = System.currentTimeMillis();
    }

    public void rezeroAllWheels() {
        for (int i = 0; i < 4; i++) {
            wheels[i].resetToAbsolute();
        }
    }
    
    public void runSysIdRoutine(Voltage voltage) {
        for (int i = 0; i < 4; i++) {
            wheels[i].runSysIdRoutine(voltage.in(Units.Volts));
        }
    }

    public void sysIdLogMotors(SysIdRoutineLog log) {
        for (int i = 0; i < 4; i++) {
            wheels[i].log();
        }
    }

        public void resetGyro() {
        gyro.zeroGyro();
    }

    public Command resetGyroCommand() {
        return runOnce(() -> {
            resetGyro();
        });
    }

    public double getAngleVelo() {
        return 1000 * (getRobotPose().getRotation().getRadians() - prevPose.getRotation().getRadians())
                / (System.currentTimeMillis() - prevPoseTimeStamp); // in radians per milisecond
    }

    public double getDriveVelo() {
        return 1000 * (getRobotPose().getTranslation().getNorm() - prevPose.getTranslation().getNorm())
                / (System.currentTimeMillis() - prevPoseTimeStamp); // in meters per milisecond
    }

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
