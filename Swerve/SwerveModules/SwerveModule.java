package POPLib.Swerve.SwerveModules;

import com.ctre.phoenix6.hardware.CANcoder;

import POPLib.SmartDashboard.PIDTuning;
import POPLib.Swerve.CTREModuleState;
import POPLib.Swerve.SwerveConstants.SwerveModuleConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.DimensionlessUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearAccelerationUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.VelocityUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import org.littletonrobotics.junction.Logger;

public abstract class SwerveModule implements SwerveModuleIO{
    public SwerveModuleConstants swerveModuleConstants;
    protected CANcoder angleEncoder;
    protected Rotation2d lastAngle;

    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    
    private final Alert driveDisconnectedAlert;
    private final Alert turnDisconnectedAlert;
    private final Alert turnEncoderDisconnectedAlert;

    protected LinearVelocity lastVelo;
    protected Time lastVeloTime;

    public SwerveModule(SwerveModuleConstants moduleConstants) {
        angleEncoder = moduleConstants.getCanCoder();
        this.swerveModuleConstants = moduleConstants;
        lastAngle = Rotation2d.fromDegrees(0);

        lastVelo = Units.MetersPerSecond.of(0.0);
        lastVeloTime = Units.Seconds.of(Timer.getFPGATimestamp());

        // Create Alerts for Disconnected components
        driveDisconnectedAlert =
            new Alert(
                "Disconnected drive motor on module " + Integer.toString(moduleConstants.moduleNumber) + ".",
                AlertType.kError);
        turnDisconnectedAlert =
            new Alert(
                "Disconnected turn motor on module " + Integer.toString(moduleConstants.moduleNumber) + ".", AlertType.kError);
        turnEncoderDisconnectedAlert =
            new Alert(
                "Disconnected turn encoder on module " + Integer.toString(moduleConstants.moduleNumber) + ".",
                AlertType.kError);
    }

    // Resets swerve module to the cancoder angle
    abstract public void resetToAbsolute();

    // Sets swerve module to swerve module state
    abstract protected void applySwerveModuleState(double velocityMPS, Rotation2d angleRadians);

    abstract protected Angle getAngle();
    abstract protected Distance getPosition();
    abstract protected LinearVelocity getVelocity();
    abstract protected Voltage getDriveVoltage();

    public void logSysId(SysIdRoutineLog log) {
        log.motor("Drive " + swerveModuleConstants.moduleNumber)
            .linearPosition(getPosition())
            .linearVelocity(getVelocity())
            .voltage(getDriveVoltage());
    }

    Rotation2d getRotation2dAngle() {
        return new Rotation2d(getAngle());
    }

    public Angle getPositionAngle() {
        return Units.Rotations.of(getPosition().divide(SwerveModuleConstants.wheelCircumference).magnitude());
    }

    public void log() {
        if (swerveModuleConstants.swerveTuningMode) {
            double angle = getAngle().in(Units.Degrees);
            putNumber("Module Angle", angle);
            putNumber("Normalized Module Angle", MathUtil.inputModulus(angle, 0.0, 360.0));
            putNumber("Drive Velo", getVelocity().in(Units.MetersPerSecond));
            putNumber("CanCoder Angle", getAbsoluteAngleDegrees());
        }
    }

    public void putNumber(String key, double value) {
        SmartDashboard.putNumber(key + " " + swerveModuleConstants.moduleNumber, value);
    }

    abstract public void updatePID(PIDTuning angle, PIDTuning drive);

    public Rotation2d getCanCoder() {
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValue().in(Units.Rotations));
    }

    public double getAbsoluteAngleDegrees() {
        return MathUtil.inputModulus(getCanCoder().getDegrees(), 0, 360);
    }

    public SwerveModuleState getState() {
         return new SwerveModuleState(getVelocity(), getRotation2dAngle());
    }

    public SwerveModulePosition getPose() {
        return new SwerveModulePosition(getPosition(), getRotation2dAngle());
    }

    public void setDesiredState(SwerveModuleState state) {
        // Prevents angle motor from turning further than it needs to. 
        // E.G. rotating from 10 to 270 degrees CW vs CCW.
        state = CTREModuleState.optimize(state, lastAngle);

        Rotation2d angle = Math.abs(state.speedMetersPerSecond) <= swerveModuleConstants.moduleInfo.maxSpeed.in(Units.MetersPerSecond) * (0.01)
            ? lastAngle
            :  state.angle;

        lastAngle = angle;

        if (swerveModuleConstants.swerveTuningMode) {
            putNumber("Target Drive Velo", state.speedMetersPerSecond);
            putNumber("Target Angle", angle.getDegrees());
        }
    
        applySwerveModuleState(state.speedMetersPerSecond, angle);
    }

    public abstract void runSysIdRoutine(double voltage);

    public LinearVelocity accelLimit(LinearVelocity newVelocity) {
        LinearVelocity velocityChange = newVelocity.minus(lastVelo);
        Time ellapsedTime = Units.Seconds.of(Timer.getFPGATimestamp()).minus(lastVeloTime);

        // TODO: Update
        // newVelocity = lastVelo.plus( 
        //     (
        //         Math.max(
        //             velocityChange.div(ellapsedTime).abs(Units.MetersPerSecondPerSecond), 
        //             swerveModuleConstants.moduleInfo.maxAcceleration.times(
        //                 1 - lastVelo.div(swerveModuleConstants.moduleInfo.maxSpeed).magnitude()
        //             ).in(Units.MetersPerSecondPerSecond)
        //         ) 
        //         * ellapsedTime.in(Units.Seconds) * (velocityChange.lt(Units.MetersPerSecond.of(0)) ? -1 : 1)));

        lastVelo = newVelocity;
        lastVeloTime.plus(ellapsedTime);

        return newVelocity;
    }

    @Override
    public void updateInputs(ModuleIOInputsAutoLogged inputs) {
        // Yet to be implemented
        

        
    }

    public void periodic() {
        updateInputs(inputs);
        Logger.processInputs("Drive/Module" + Integer.toString(swerveModuleConstants.moduleNumber), inputs);

        // Update alerts
        driveDisconnectedAlert.set(!inputs.driveConnected);
        turnDisconnectedAlert.set(!inputs.turnConnected);
        turnEncoderDisconnectedAlert.set(!inputs.turnEncoderConnected);
    }
}
