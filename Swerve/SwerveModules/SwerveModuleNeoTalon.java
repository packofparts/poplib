package POPLib.Swerve.SwerveModules;

import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import POPLib.SmartDashboard.PIDTuning;
import POPLib.Swerve.SwerveConstants.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
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
    protected void applySwerveModuleState(double velocityMPS, Rotation2d angleRadians) {
        driveMotor.setControl(drivePID.withVelocity(velocityMPS)); 
        anglePID.setReference(angleRadians.getDegrees(), SparkMax.ControlType.kPosition);

        if (swerveModuleConstants.swerveTuningMode) {
            SmartDashboard.putNumber("Target Drive Velocity: " + swerveModuleConstants.moduleNumber, velocityMPS);
            SmartDashboard.putNumber("Target Relative Encoder Angle " + swerveModuleConstants.moduleNumber, angleRadians.getDegrees());
        }
    }

    @Override
    protected Angle getAngle() {
        return Units.Rotations.of(angleEncoder.getPosition());
    }

    @Override
    protected Distance getPosition() {
        return Units.Meters.of(driveMotor.getPosition().getValueAsDouble());
    }

    @Override
    protected LinearVelocity getVelocity() {
        return Units.MetersPerSecond.of(driveMotor.getVelocity().getValueAsDouble());
    }

    @Override
    public void updatePID(PIDTuning angle, PIDTuning drive) {
        angle.updatePID(angleMotor);
        drive.updatePID(driveMotor);
    }
}