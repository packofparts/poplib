package POPLib.Swerve.SwerveModules;

import com.ctre.phoenix6.hardware.CANcoder;

import POPLib.SmartDashboard.PIDTuning;
import POPLib.Swerve.CTREModuleState;
import POPLib.Swerve.SwerveConstants.SwerveModuleConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;

public abstract class SwerveModule {
    public SwerveModuleConstants swerveModuleConstants;
    protected CANcoder angleEncoder;
    protected Rotation2d lastAngle;
    
    protected double lastVelo;
    protected double lastVeloTime;

    public SwerveModule(SwerveModuleConstants moduleConstants) {
        angleEncoder = moduleConstants.getCanCoder();
        this.swerveModuleConstants = moduleConstants;
        lastAngle = Rotation2d.fromDegrees(0);

        lastVelo = 0;
        lastVeloTime = Timer.getFPGATimestamp();
    }

    // Resets swerve module to the cancoder angle
    abstract public void resetToAbsolute();

    // Sets swerve module to swerve module state
    abstract protected void applySwerveModuleState(double velocityMPS, Rotation2d angleRadians);

    abstract protected Rotation2d getEncoderAngle();
    abstract protected double getPositionMeter();
    abstract protected double getVelocityMeter();

    public double getPositionRotationRadians() {
        return (getPositionMeter() / SwerveModuleConstants.wheelCircumference) * 2 * Math.PI;
    }

    public void log() {}

    abstract public void updatePID(PIDTuning angle, PIDTuning drive);

    public Rotation2d getCanCoder() {
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValue());
    }

    public double getAbsoluteAngleDegrees() {
        return MathUtil.inputModulus(getCanCoder().getDegrees(), 0, 360);
    }

    public SwerveModuleState getState() {
         return new SwerveModuleState(getVelocityMeter(), getEncoderAngle());
    }

    public SwerveModulePosition getPose() {
        return new SwerveModulePosition(getPositionMeter(), getEncoderAngle());
    }

    public void setDesiredState(SwerveModuleState state) {
        // Prevents angle motor from turning further than it needs to. 
        // E.G. rotating from 10 to 270 degrees CW vs CCW.
        state = CTREModuleState.optimize(state, lastAngle);

        Rotation2d angle = Math.abs(state.speedMetersPerSecond) <= swerveModuleConstants.moduleInfo.maxSpeed * 0.01
            ? lastAngle
            :  state.angle;

        lastAngle = angle;

        applySwerveModuleState(state.speedMetersPerSecond, angle);
    }

    public double accelLimit(double newVelocity) {
        double velocityChange = newVelocity - lastVelo;
        double ellapsedTime = Timer.getFPGATimestamp() - lastVeloTime;

        newVelocity = lastVelo + Math.max(
                Math.abs(velocityChange / ellapsedTime), 
                swerveModuleConstants.moduleInfo.maxAngularAcceleration * (1 - (lastVelo / swerveModuleConstants.moduleInfo.maxSpeed))
            ) * ellapsedTime * (velocityChange < 0 ? -1 : 1);

        lastVelo = newVelocity;
        lastVeloTime += ellapsedTime;

        return newVelocity;
    }
}
