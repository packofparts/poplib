package POPLib.Swerve.SwerveModules;

import com.ctre.phoenix6.hardware.CANcoder;
import POPLib.SmartDashboard.PIDTuning;
import POPLib.Swerve.CTREModuleState;
import POPLib.Swerve.SwerveConstants.SwerveModuleConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public abstract class SwerveModule {
    public SwerveModuleConstants swerveModuleConstants;
    protected CANcoder angleEncoder;
    protected Rotation2d lastAngle;

    private final ModuleIOInputsAutoLogged inputs;

    protected LinearVelocity lastVelo;
    protected Time lastVeloTime;

    @AutoLog
    public static class ModuleIOInputs {

      public SwerveModuleState swerveState = new SwerveModuleState();
      public SwerveModulePosition swervePose = new SwerveModulePosition();

      public Distance driveMotorDistance = Units.Meters.zero();
      public LinearVelocity driveLinearVelocity = Units.MetersPerSecond.zero();
      public Current driveCurrent = Units.Amps.zero();
  
      public Rotation2d turnEncoderPosition = new Rotation2d();
      public Rotation2d turnMotorPosition = new Rotation2d();
      public AngularVelocity turnAngularVelocity = Units.RadiansPerSecond.zero();
      public Current turnCurrent = Units.Amps.zero();
    }

    public SwerveModule(SwerveModuleConstants moduleConstants) {
        angleEncoder = moduleConstants.getCanCoder();
        this.swerveModuleConstants = moduleConstants;
        lastAngle = Rotation2d.fromDegrees(0);

        inputs = new ModuleIOInputsAutoLogged();

        lastVelo = Units.MetersPerSecond.of(0.0);
        lastVeloTime = Units.Seconds.of(Timer.getFPGATimestamp());
    }

    // Resets swerve module to the cancoder angle
    abstract public void resetToAbsolute();

    // Sets swerve module to swerve module state
    abstract protected void applySwerveModuleState(double velocityMPS, Rotation2d angleRadians);

    abstract protected Angle getAngle();
    abstract protected Distance getPosition();
    abstract protected LinearVelocity getVelocity();
    abstract protected AngularVelocity getTurnAngularVelocity();
    abstract protected Voltage getDriveVoltage();
    abstract protected Voltage getTurnVoltage();
    abstract protected Current getDriveCurrent();
    abstract protected Current getTurnCurrent();

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

     public LinearVelocity accelLimit(LinearVelocity newVelo) {
        LinearVelocity veloChange = newVelo.minus(lastVelo);
        Time elapsedTime = Units.Seconds.of(Timer.getFPGATimestamp()).minus(lastVeloTime);

        LinearAcceleration maxAcceleration = swerveModuleConstants.moduleInfo.maxAcceleration.times(
            1 - lastVelo.divide(swerveModuleConstants.moduleInfo.maxSpeed).magnitude()
        );

        if (veloChange.divide(elapsedTime).gt(maxAcceleration)) {
            newVelo = lastVelo.plus(
                maxAcceleration.times(elapsedTime)
                .times(veloChange.lt(Units.MetersPerSecond.of(0)) ? -1 : 1)                    
            );
        }

        lastVelo = newVelo;
        lastVeloTime = lastVeloTime.plus(elapsedTime);

        return newVelo;
    }

    public void updateInputs(ModuleIOInputsAutoLogged inputs) {
        // Update State and Pose
        inputs.swerveState = getState();
        inputs.swervePose = getPose();
        
        // Update Drive Inputs
        inputs.driveMotorDistance = getPosition();
        inputs.driveLinearVelocity = getVelocity();
        inputs.driveCurrent = getDriveCurrent();

        // Update Current Inputs
        inputs.turnEncoderPosition = getRotation2dAngle();
        inputs.turnMotorPosition = Rotation2d.fromRotations(getAngle().magnitude());
        inputs.turnAngularVelocity = getTurnAngularVelocity();
        inputs.turnCurrent = getTurnCurrent();
    }

    public void periodic() {
        updateInputs(inputs);
        Logger.processInputs("Drive/Module" + swerveModuleConstants.moduleNumber, inputs);
    }
}
