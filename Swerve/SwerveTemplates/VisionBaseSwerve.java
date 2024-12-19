package POPLib.Swerve.SwerveTemplates;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import POPLib.Sensors.Camera.Camera;
import POPLib.Sensors.Camera.CameraConfig;
import POPLib.Sensors.Gyro.Gyro;
import POPLib.Swerve.SwerveModules.SwerveModule;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public abstract class VisionBaseSwerve extends BaseSwerve {
    protected final SwerveDrivePoseEstimator odom;
    private final SwerveDriveKinematics kinematics;
    private ArrayList<Camera> cameras;

    public VisionBaseSwerve(SwerveModule[] swerveMods, Gyro gyro, SwerveDriveKinematics kinematics,
            Matrix<N3, N1> stateStdDevs, Matrix<N3, N1> visionMeasurementStdDevs, List<CameraConfig> cameraConfigs) {
        super(swerveMods, gyro);
        this.kinematics = kinematics;

        this.odom = new SwerveDrivePoseEstimator(
                kinematics,
                getGyro().getNormalizedRotation2dAngle(),
                getPose(),
                new Pose2d(0, 0, getGyro().getNormalizedRotation2dAngle()),
                stateStdDevs,
                visionMeasurementStdDevs);

        setPrevPose(this.odom.getEstimatedPosition());

        this.cameras = new ArrayList<Camera>();
        for (int i = 0; i < cameraConfigs.size(); i++) {
            cameras.add(new Camera(cameraConfigs.get(i)));
        }
    }

    public VisionBaseSwerve(SwerveModule[] swerveMods, Gyro gyro, SwerveDriveKinematics kinematics, List<CameraConfig> cameraConfigs) {
        this(swerveMods, gyro, kinematics, VecBuilder.fill(0.1, 0.1, 0.05),
                VecBuilder.fill(0.9, 0.9, 0.9), cameraConfigs);
    }

    public void updateVisionPoses() {
        for (int i = 0; i < cameras.size(); i++) {
            Optional<EstimatedRobotPose> estPose = cameras.get(i).getEstimatedPose(getOdomPose());
            if (estPose.isPresent()) {
                odom.addVisionMeasurement(estPose.get().estimatedPose.toPose2d(), 
                estPose.get().timestampSeconds, cameras.get(i).getVisionStdDevs());
            }
        }
    }

    @Override
    public void driveRobotOriented(Translation2d vector, double rot) {
        // vector = accelrationLimit(vector);

        SwerveModuleState[] states = kinematics
                .toSwerveModuleStates(new ChassisSpeeds(vector.getX(), vector.getY(), rot));

        driveRobotOriented(states);
    }

    @Override
    public void driveChassis(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);
        driveRobotOriented(states);
    }

    public void setOdomPose(Pose2d pose) {
        odom.resetPosition(pose.getRotation(), getPose(), pose);
        setGyro(pose);
    }

    @Override
    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getStates());
    }

    @Override
    public Pose2d getOdomPose() {
        return odom.getEstimatedPosition();
    }

    @Override
    public void periodic() {
        super.periodic();
        odom.update(getGyro().getNormalizedRotation2dAngle(), getPose());
    }
}
