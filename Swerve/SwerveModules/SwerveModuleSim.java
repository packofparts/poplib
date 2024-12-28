package POPLib.Swerve.SwerveModules;

import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
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

public class SwerveModuleSim extends SwerveModule{
    private DCMotorSim angleSim;
    private DCMotorSim driveSim;

    private static final DCMotor DRIVE_GEARBOX = DCMotor.getKrakenX60Foc(1);
    private static final DCMotor TURN_GEARBOX = DCMotor.getKrakenX60Foc(1);

    private VelocityDutyCycle drivePID;
    private VoltageOut driveVoltage;
    private PositionDutyCycle anglePID;

    public static final double driveInertia = 0.001;
    public static final double steerInertia = 0.00001;
    private static final double driveGearRatio = 7.363636364;
    private static final double steerGearRatio = 12.8;

    public SwerveModuleSim(SwerveModuleConstants moduleConstants) {
        super(moduleConstants);

        driveSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DRIVE_GEARBOX, driveInertia, driveGearRatio),
            DRIVE_GEARBOX);
        angleSim =
            new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                    TURN_GEARBOX, steerInertia, steerGearRatio),
                TURN_GEARBOX);

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
        
    }

    @Override
    protected Angle getAngle() {
        return null;
    }

    @Override
    protected Distance getPosition() {
        return null;
    }

    @Override
    protected LinearVelocity getVelocity() {
        // Convert from default units Rotations/Sec to M/S
        return null;
    }

    @Override
    protected AngularVelocity getTurnAngularVelocity() {
        // Convert from default units Rotations/Sec to Radians/Sec
        return null;
    }

    @Override
    protected Current getDriveCurrent() {
        return null;
    }

    @Override
    protected Current getTurnCurrent() {
        return null;
    }

    @Override
    public void updatePID(PIDTuning angle, PIDTuning drive) {
        
    }

    @Override
    public void runSysIdRoutine(double voltage) {
        
    }

    @Override
    protected Voltage getDriveVoltage() {
        return null;
    }

    @Override
    protected Voltage getTurnVoltage() {
        return null;
    }
}
