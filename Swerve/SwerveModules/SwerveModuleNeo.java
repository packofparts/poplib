package POPLib.Swerve.SwerveModules;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;
import POPLib.SmartDashboard.PIDTuning;
import POPLib.Swerve.SwerveConstants.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;

public class SwerveModuleNeo extends SwerveModule {
    private final SparkMax driveMotor;
    private final SparkMax angleMotor;


    public SwerveModuleNeo(SwerveModuleConstants constants) {
        super(constants);

        driveMotor = constants.getDriveNeo();
        angleMotor = constants.getAngleNeo();
    }

    @Override
    protected void applySwerveModuleState(double velocityMPS, Rotation2d angleRadians) {
        angleMotor.getClosedLoopController().setReference(angleRadians.getRotations(), SparkMax.ControlType.kPosition);
        driveMotor.getClosedLoopController().setReference(velocityMPS, SparkMax.ControlType.kVelocity);
    }

    @Override
    public void resetToAbsolute() {
        angleEncoder.setPosition(getCanCoder().getRotations()); 
    }

    @Override
    protected Angle getAngle() {
        return Units.Rotations.of(angleMotor.getEncoder().getPosition());
    }

    @Override
    protected Distance getPosition() {
        return Units.Meters.of(driveMotor.getEncoder().getPosition());
    }

    @Override
    protected LinearVelocity getVelocity() {
        return Units.MetersPerSecond.of(driveMotor.getEncoder().getVelocity());
    }

    @Override
    public void updatePID(PIDTuning angle, PIDTuning drive) {
        angle.updatePID(angleMotor);
        drive.updatePID(driveMotor);
    }

    @Override
    public void runSysIdRoutine(double voltage) {
        angleMotor.getClosedLoopController().setReference(0.0, SparkMax.ControlType.kPosition);
        driveMotor.setVoltage(voltage);
    }

    @Override
    protected Voltage getDriveVoltage() {
        // TOOD: Verify correctness
        return Units.Volts.of(driveMotor.getBusVoltage());
    }

    public void updateInputs(ModuleIOInputsAutoLogged inputs) {
            // Update drive inputs
        sparkStickyFault = false;
        ifOk(driveSpark, driveEncoder::getPosition, (value) -> inputs.drivePositionRad = value);
        ifOk(driveSpark, driveEncoder::getVelocity, (value) -> inputs.driveVelocityRadPerSec = value);
        ifOk(
            driveSpark,
            new DoubleSupplier[] {driveSpark::getAppliedOutput, driveSpark::getBusVoltage},
            (values) -> inputs.driveAppliedVolts = values[0] * values[1]);
        ifOk(driveSpark, driveSpark::getOutputCurrent, (value) -> inputs.driveCurrentAmps = value);
        inputs.driveConnected = driveConnectedDebounce.calculate(!sparkStickyFault);

        // Update turn inputs
        sparkStickyFault = false;
        ifOk(
            turnSpark,
            turnEncoder::getPosition,
            (value) -> inputs.turnPosition = new Rotation2d(value).minus(zeroRotation));
        ifOk(turnSpark, turnEncoder::getVelocity, (value) -> inputs.turnVelocityRadPerSec = value);
        ifOk(
            turnSpark,
            new DoubleSupplier[] {turnSpark::getAppliedOutput, turnSpark::getBusVoltage},
            (values) -> inputs.turnAppliedVolts = values[0] * values[1]);
        ifOk(turnSpark, turnSpark::getOutputCurrent, (value) -> inputs.turnCurrentAmps = value);
        inputs.turnConnected = turnConnectedDebounce.calculate(!sparkStickyFault);

        // Update odometry inputs
        inputs.odometryTimestamps =
            timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryDrivePositionsRad =
            drivePositionQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryTurnPositions =
            turnPositionQueue.stream()
                .map((Double value) -> new Rotation2d(value).minus(zeroRotation))
                .toArray(Rotation2d[]::new);
        timestampQueue.clear();
        drivePositionQueue.clear();
        turnPositionQueue.clear();


        
    }
}
