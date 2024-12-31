package POPLib.Swerve.SwerveModules;

import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import POPLib.SmartDashboard.PIDTuning;
import POPLib.Swerve.SwerveConstants.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * Falcon Swerve Module.
 */
public class SwerveModuleSim extends SwerveModule {
    private DCMotorSim driveMotor;
    private DCMotorSim angleMotor;

    private VelocityDutyCycle drivePID;
    private VoltageOut driveVoltage;
    private PositionDutyCycle anglePID;

    public SwerveModuleSim(SwerveModuleConstants moduleConstants) {
        super(moduleConstants);

        driveMotor = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(-1), -1, -1),
                                    DCMotor.getKrakenX60Foc(-1));

        angleMotor = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(-1), -1, -1),
                                    DCMotor.getKrakenX60Foc(-1));

        resetToAbsolute();

        lastAngle = getPose().angle;

        drivePID = new VelocityDutyCycle(0); 
        driveVoltage = new VoltageOut(0.0);
        anglePID = new PositionDutyCycle(0); 
    }

    @Override
    public void resetToAbsolute() {
    }

    @Override
    protected void applySwerveModuleState(double velocityMPS, Rotation2d angle) {
        //TODO
        //driveMotor.setInputVoltage(MathUtil.clamp(driveAppliedVolts, -12.0, 12.0));
        //angleMotor.setInputVoltage(MathUtil.clamp(turnAppliedVolts, -12.0, 12.0));
        //driveMotor.update(0.02);
        //angleMotor.update(0.02);
    }

    @Override
    protected Angle getAngle() {
        return angleMotor.getAngularPosition();
    }

    @Override
    protected Distance getPosition() {
        return Units.Meter.of(driveMotor.getAngularPositionRotations());
    }

    @Override
    protected LinearVelocity getVelocity() {
        // Convert from default units Rotations/Sec to M/S
        return Units.MetersPerSecond.of(driveMotor.getAngularVelocityRPM() * 
        SwerveModuleConstants.wheelCircumference.magnitude() * 60);
    }

    @Override
    protected AngularVelocity getTurnAngularVelocity() {
        return angleMotor.getAngularVelocity();
    }

    @Override
    protected Current getDriveCurrent() {
        //TODO
        return null;
        //return Units.Amps.of(driveMotor.getStatorCurrent().getValueAsDouble());
    }

    @Override
    protected Current getTurnCurrent() {
        //TODO
        return null;
        //return Units.Amps.of(angleMotor.getStatorCurrent().getValueAsDouble());
    }

    @Override
    public void updatePID(PIDTuning angle, PIDTuning drive) {
        //TODO
    }

    @Override
    public void runSysIdRoutine(double voltage) {
        //TODO
        // angleMotor.setControl(anglePID.withPosition(0.0)); 
        // driveMotor.setControl(driveVoltage.withOutput(voltage));
    }

    @Override
    protected Voltage getDriveVoltage() {
        //TODO
        return null;
    }

    @Override
    protected Voltage getTurnVoltage() {
        //TODO
        return null;
    }
}
