package poplibv2.subsystems.swerve;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
    public Pose2d currPose;
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
        
        cameras = new Camera[config.cameraConfigs.length];
        for (int i = 0; i < config.cameraConfigs.length; i++) {
            cameras[i] = new Camera(config.cameraConfigs[i]);
        }

        limelights = new Limelight[config.limelightConfigs.length];
        for (int i = 0; i < config.limelightConfigs.length; i++) {
            limelights[i] = new Limelight(config.limelightConfigs[i]);
        }

        field = new Field2d();
        currPose = new Pose2d();
        driveMotorTuning = new PIDTuning("Swerve Motor Drive", new PIDConfig(), SwerveConfig.tuningEnable);
        rotMotorTuning = new PIDTuning("Swerve Motor Rotation", new PIDConfig(), SwerveConfig.tuningEnable);

        kinematics = new SwerveDriveKinematics(config.wheelPos);
        odometry = new SwerveDrivePoseEstimator(kinematics, gyro.getNormalizedRotation2dAngle(), getModulePositions(), currPose);
    }

    @Override
    public void periodic() {
        updatePID();
        updateOdomUsingVision();
    }

    private void updatePID() {
        for (int i = 0; i < 4; i++) {
            wheels[i].updatePID(driveMotorTuning, rotMotorTuning);
        }
    }

    private void updateOdomUsingVision() {
        for (int i = 0; i < cameras.length; i++) {
            Optional<EstimatedRobotPose> estimatedPose = cameras[i].getEstimatedPose(currPose);
            if (estimatedPose.isPresent()) {
                odometry.addVisionMeasurement(estimatedPose.get().estimatedPose.toPose2d(), 
                                              estimatedPose.get().timestampSeconds, 
                                              cameras[i].getVisionStdDevs());
            }
        }
    }

    private void driveRobotOriented(ChassisSpeeds speeds) {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        for (int i = 0; i < 4; i++) {
            wheels[i].setDesiredState(states[i]);
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

}
