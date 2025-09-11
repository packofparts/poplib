package poplibv2.subsystems.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import poplibv2.control_systems.PIDTuning;
import poplibv2.motors.Motor;
import poplibv2.sensors.absolute_encoder.CANCoder;
import poplibv2.subsystems.swerve.setup.SwerveConfig;
import poplibv2.subsystems.swerve.setup.SwerveModuleConfig;

public class SwerveModule {
    Motor driveMotor;
    Motor rotMotor;
    CANCoder encoder;
    int id;
    LinearVelocity maxSpeed;

    Rotation2d lastAngle;
    LinearVelocity lastVelo;
    Time lastVeloTime;

    /**
     * Creates a new swerve module (wheel) using a config
     * Also sets the wheel of the swerve module to be straight 
     * during intialization, assuming the absolute encoder offset 
     * is correct.
     * @param config
     */
    public SwerveModule(SwerveModuleConfig config) {
        driveMotor = new Motor(config.driveMotorConfig);
        rotMotor = new Motor(config.rotMotorConfig);
        encoder = new CANCoder(config.canCoderConfig);
        maxSpeed = config.maxSpeed;
        resetToAbsolute();
        // lastAngle = getPose().rotation();
    }

    /**
     * "Zeros out" the motors using an absolute encoder.
     * Call in the intializer
     */
    public void resetToAbsolute() {
        rotMotor.setTargetPosition(encoder.getPosition().getRotations());
    }

    /**
     * Drives the motors toward a certain state
     * @param velocity The drive motor velocity to apply (in RPM)
     * @param rotation The wheel position to apply (in rotations)
     */
    public void applySwerveModuleState(double velocity, Rotation2d rotation) {
        driveMotor.setTargetVelocity(velocity);
        rotMotor.setTargetPosition(rotation.getRotations());
    }

    /**
     * Gets the angle of the wheel
     * @return The angle as an Angle object
     */
    public Angle getWheelAngle() {
        return Units.Rotations.of(rotMotor.getPosition());
    }

    /**
     * Gets the amount of rotations the drive motor has driven the wheel
     * @return The rotations as an Angle object
     */
    public Angle getDriveRotations() {
        return Units.Rotations.of(driveMotor.getPosition());
    }

    /**
     * Gets the angluar velocity of the wheel
     * @return The AngularVelocity in rotations per second
     */
    public AngularVelocity getWheelVelocity() {
        return Units.RPM.of(driveMotor.getVelocity());
    }

    /**
     * Gets the amount of meters this wheel has driven.
     * @return the meters as a Distance object
     */
    public Distance getDrivePosition() {
        return Units.Meters.of(
            getDriveRotations().in(Units.Rotations) * SwerveConfig.wheelCircumference.in(Units.Meters)
        );
    }

    /**
     * Gets the velocity of the wheel in MPS
     * @return the velocity as a LinearVelocity object
     */
    public LinearVelocity getVelocity() {
        return Units.MetersPerSecond.of(
            getWheelVelocity().in(Units.RotationsPerSecond) * 
            SwerveConfig.wheelCircumference.in(Units.Meters)
        );
    }

    /**
     * Gets the voltage of the drive motor
     * @return
     */
    public Voltage getDriveVoltage() {
        return driveMotor.getVoltage();
    }

    /**
     * For use in deteremining kF
     * @param log
     */
    public void logSysId(SysIdRoutineLog log) {
        log.motor("Drive " + id).linearPosition(getDrivePosition())
        .linearVelocity(getVelocity()).voltage(getDriveVoltage());
    }

    /**
     * Gets the angle of the wheel as a Rotation2d object
     * @return
     */
    Rotation2d getRotation2dAngle() {
        return new Rotation2d(getWheelAngle());
    }

    /**
     * Logs important values
     */
    public void log() {
        double angle = getWheelAngle().in(Units.Degrees);
        putNumber("Module Angle", angle);
        putNumber("Normalized Module Angle", MathUtil.inputModulus(angle, 0.0, 360.0));
        putNumber("Drive Velo", getVelocity().in(Units.MetersPerSecond));
        putNumber("CanCoder Angle", getAbsoluteAngleDegrees());
    }

    private void putNumber(String key, double value) {
        SmartDashboard.putNumber(key + " " + id, value);
    }

    /**
     * Updates the motor pid
     * @param drive
     * @param rot
     */
    public void updatePID(PIDTuning drive, PIDTuning rot) {
        driveMotor.changePID(drive.generatePIDConfig());
        rotMotor.changePID(rot.generatePIDConfig());
    }

    /**
     * Normalizes the abs encoder reading and returns it in degrees
     * @return
     */
    public double getAbsoluteAngleDegrees() {
        return MathUtil.inputModulus(encoder.getPosition().getDegrees(), 0, 360);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity(), getRotation2dAngle());
    }

    public SwerveModulePosition getPose() {
        return new SwerveModulePosition(getDrivePosition(), getRotation2dAngle());
    }

    public void setDesiredState(SwerveModuleState state) {
        state.optimize(getRotation2dAngle());
        state.speedMetersPerSecond *= state.angle.minus(getRotation2dAngle()).getCos();

        putNumber("Target Drive Velo", state.speedMetersPerSecond);
        putNumber("Target Angle", state.angle.getDegrees());
    
        double metersPerMinute = state.speedMetersPerSecond * 60.0;
        double rotationsPerMinute = metersPerMinute / SwerveConfig.wheelCircumference.in(Units.Meters);
        applySwerveModuleState(rotationsPerMinute, state.angle);
    }

    public void runSysIdRoutine(double voltage) {
        driveMotor.setTargetVelocity(100, voltage);
    }
}
