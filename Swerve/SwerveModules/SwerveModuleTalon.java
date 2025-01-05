package POPLib.Swerve.SwerveModules;

import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import POPLib.SmartDashboard.PIDTuning;
import POPLib.Swerve.SwerveConstants.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

/**
 * Falcon Swerve Module.
 */
public class SwerveModuleTalon extends SwerveModule {
    private TalonFX angleMotor;
    private TalonFX driveMotor;

    private VelocityDutyCycle drivePID;
    private VoltageOut driveVoltage;
    private PositionDutyCycle anglePID;

    public SwerveModuleTalon(SwerveModuleConstants moduleConstants) {
        super(moduleConstants);

        angleMotor = moduleConstants.getAngleFalcon();
        driveMotor = moduleConstants.getDriveFalcon();

        resetToAbsolute();

        lastAngle = getPose().angle;

        drivePID = new VelocityDutyCycle(0); 
        driveVoltage = new VoltageOut(0.0);
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
        angleMotor.setControl(anglePID.withPosition(angle.getRotations()));
    }

    @Override
    protected Angle getAngle() {
        return angleMotor.getPosition().getValue();
    }

    @Override
    protected Angle getDriveAngle() {
        return driveMotor.getPosition().getValue();
    }

    @Override
    protected AngularVelocity getDriveAngularVelocity() {
        return driveMotor.getVelocity().getValue();
    }

    @Override
    public void updatePID(PIDTuning angle, PIDTuning drive) {
        angle.updatePID(angleMotor);
        drive.updatePID(driveMotor);
    }

    @Override
    public void runSysIdRoutine(double voltage) {
        angleMotor.setControl(anglePID.withPosition(0.0)); 
        driveMotor.setControl(driveVoltage.withOutput(voltage));
    }

    @Override
    protected Voltage getDriveVoltage() {
        return driveMotor.getMotorVoltage().getValue();
    }
}
