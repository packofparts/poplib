package POPLib.Swerve.SwerveModules;

import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import POPLib.SmartDashboard.PIDTuning;
import POPLib.Swerve.SwerveConstants.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

/**
 * Falcon Swerve Module.
 */
public class SwerveModuleTalon extends SwerveModule {
    private TalonFX angleMotor;
    private TalonFX driveMotor;

    private VelocityDutyCycle drivePID;
    private PositionDutyCycle anglePID;

    public SwerveModuleTalon(SwerveModuleConstants moduleConstants) {
        super(moduleConstants);

        angleMotor = moduleConstants.getAngleFalcon();
        driveMotor = moduleConstants.getDriveFalcon();

        resetToAbsolute();

        lastAngle = getPose().angle;

        drivePID = new VelocityDutyCycle(0); 
        anglePID = new PositionDutyCycle(0); 
    }

    @Override
    public void resetToAbsolute() {
        angleMotor.setPosition(getCanCoder().getRotations());
        lastAngle = getCanCoder();
    }

    @Override
    protected void applySwerveModuleState(double velocityMPS, Rotation2d angle) {
        driveMotor.setControl(drivePID.withVelocity(velocityMPS)); 
        angleMotor.setControl(anglePID.withPosition(angle.getDegrees()));
    }

    @Override
    protected Angle getAngle() {
        return angleMotor.getPosition().getValue();
    }

    @Override
    protected Distance getPosition() {
        return Units.Meter.of(driveMotor.getPosition().getValueAsDouble());
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
