package poplibv2.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import poplibv2.control_systems.PIDConfig;
import poplibv2.control_systems.PIDTuning;
import poplibv2.misc.TunableNumber;
import poplibv2.motors.FollowerConfig;
import poplibv2.motors.Motor;
import poplibv2.motors.MotorConfig;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;

public abstract class Elevator extends SubsystemBase{
    private final TunableNumber setpoint;
    private PIDTuning tuning;
    private ElevatorFeedforward feedforward;
    private Distance rotationToMeters;
    private Motor motor;

    public Elevator(MotorConfig leadMotor, FollowerConfig[] followerMotors, ElevatorFeedforward feedforward, Distance rotationToMeters, boolean tuningMode, String subsytemName) {
        super(subsytemName);

        setpoint = new TunableNumber("Elevator Setpoint", 0, tuningMode);
        tuning = new PIDTuning("Elevator", new PIDConfig(), tuningMode);
        this.feedforward = feedforward;
        motor = new Motor(leadMotor);
        for (FollowerConfig config : followerMotors) {
            motor.addFollowerMotor(config);
        }
        this.rotationToMeters = rotationToMeters;
    }

    @Override
    public void periodic() {
        
    }

    public abstract double getError(double setpoint);

    public abstract double getEncoderPos();

    public Command moveElevator(double setPoint, double error) {
        return run(() -> setpoint.set(setPoint*rotationToMeters.in(Units.Meters))). // update this to work, math be wrong
        until(() -> getError(setPoint) < error);
    }

    public abstract Command moveUp(double speed);

    public abstract Command moveDown(double speed);

    public abstract Command stop();

    public void runSysIdRoutine(Voltage voltage) {
        motor.setVoltage(voltage.in(Units.Volts));
    }

    public void sysIdLogMotors(SysIdRoutineLog log) {
        log.motor("Elevator Motor").linearPosition(Units.Meters.of(getEncoderPos())); // update this 
    } 
}
