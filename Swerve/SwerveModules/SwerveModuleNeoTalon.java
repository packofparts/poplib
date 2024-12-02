package POPLib.Swerve.SwerveModules;

import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import POPLib.Math.Conversion;
import POPLib.SmartDashboard.PIDTuning;
import POPLib.Swerve.SwerveConstants.SwerveModuleConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModuleNeoTalon extends SwerveModule {
    private final TalonFX driveMotor;
    private final VelocityDutyCycle drivePID;

    private final SparkMax angleMotor;
    private final RelativeEncoder angleEncoder;
    private final SparkClosedLoopController anglePID;

    public SwerveModuleNeoTalon(SwerveModuleConstants constants) {
        super(constants);
        driveMotor = constants.getDriveFalcon(); 

        angleMotor = constants.getAngleNeo();
        angleEncoder = angleMotor.getEncoder();
        anglePID = angleMotor.getClosedLoopController();

        resetToAbsolute();

        drivePID = new VelocityDutyCycle(0); 
        lastAngle = getPose().angle;
    }


    @Override
    public void resetToAbsolute() {
        angleEncoder.setPosition(getCanCoder().getDegrees()); 
        lastAngle = getCanCoder();
    }

    @Override
    public void log() {
        if (swerveModuleConstants.swerveTuningMode) {
            // SmartDashboard.putNumber("Angle Current " + swerveModuleConstants.moduleNumber, angleMotor.getOutputCurrent());
            SmartDashboard.putNumber("Current Reltaive Encoder Angle " + swerveModuleConstants.moduleNumber, angleEncoder.getPosition());
            SmartDashboard.putNumber("Current Reltaive Encoder Angle Mod " + swerveModuleConstants.moduleNumber, MathUtil.inputModulus(angleEncoder.getPosition(), 0, 360));
            SmartDashboard.putNumber("Current Drive Velocity" + swerveModuleConstants.moduleNumber, Conversion.RPSToMPS(driveMotor.getVelocity().getValue().in(Units.RotationsPerSecond), SwerveModuleConstants.wheelCircumference));
            SmartDashboard.putNumber("CanCoder Angle" + swerveModuleConstants.moduleNumber, getAbsoluteAngleDegrees());
        }
    }

    @Override
    protected void applySwerveModuleState(double velocityMPS, Rotation2d angleRadians) {
        driveMotor.setControl(drivePID.withVelocity(Conversion.MPSToRPS(velocityMPS, SwerveModuleConstants.wheelCircumference))); 
        anglePID.setReference(angleRadians.getDegrees(), SparkMax.ControlType.kPosition);

        if (swerveModuleConstants.swerveTuningMode) {
            SmartDashboard.putNumber("Target Drive Velocity: " + swerveModuleConstants.moduleNumber, velocityMPS);
            SmartDashboard.putNumber("Target Relative Encoder Angle " + swerveModuleConstants.moduleNumber, angleRadians.getDegrees());
        }
    }

    @Override
    protected Rotation2d getEncoderAngle() {
        return Rotation2d.fromDegrees(angleEncoder.getPosition());
    }

    @Override
    protected double getPositionMeter() {
        return Conversion.rotationsToM(driveMotor.getPosition().getValue().in(Units.Rotations), SwerveModuleConstants.wheelCircumference);
    }

    @Override
    protected double getVelocityMeter() {
        return Conversion.RPSToMPS(driveMotor.getVelocity().getValue().in(Units.RotationsPerSecond), SwerveModuleConstants.wheelCircumference);
    }

    @Override
    public void updatePID(PIDTuning angle, PIDTuning drive) {
        angle.updatePID(angleMotor);
        drive.updatePID(driveMotor);
    }
}