package poplibv2.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import poplibv2.control_systems.PIDConfig;
import poplibv2.control_systems.PIDTuning;
import poplibv2.misc.TunableNumber;
import poplibv2.motors.FollowerConfig;
import poplibv2.motors.Motor;
import poplibv2.motors.MotorConfig;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;

public abstract class Elevator extends SubsystemBase{
    private final TunableNumber setpoint;
    private PIDTuning tuning;
    private ElevatorFeedforward feedforward;
    private TrapezoidProfile setPointCalc;
    private TrapezoidProfile.State goal;
    private TrapezoidProfile.State calculated;
    private Distance rotationToMeters;
    private Motor motor;
    private double maxExtension;
    private final double loopTime = 0.02;

    /**
     * Creates a new Elevator
     * @param leadMotor the lead motor config
     * @param followerMotors an array of FollowerConfigs for any follower motors
     * @param feedforward the feedforward calculator 
     * @param rotationToMeters how many meters the elevator goes up when the motors are driven 1 rotation
     * @param maxVelocity in Meters/s
     * @param maxAcceleration in Meters/s/s
     * @param tuningMode whether or not to allow PID and setpoint tuning
     * @param subsytemName the name of the subsystem, for logging
     */
    public Elevator(MotorConfig leadMotor, FollowerConfig[] followerMotors, ElevatorFeedforward feedforward, 
    Distance rotationToMeters, double maxVelocity, double maxAcceleration, double maxExtension, boolean tuningMode, String subsytemName) {
        super(subsytemName);

        setpoint = new TunableNumber("Elevator Setpoint", 0, tuningMode);
        tuning = new PIDTuning("Elevator", new PIDConfig(), tuningMode);
        this.feedforward = feedforward;
        motor = new Motor(leadMotor);
        for (FollowerConfig config : followerMotors) {
            motor.addFollowerMotor(config);
        }
        this.rotationToMeters = rotationToMeters;
        this.setPointCalc = new TrapezoidProfile(new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
        this.maxExtension = maxExtension;
    }

    @Override
    public void periodic() {
        motor.changePID(tuning.generatePIDConfig());     // update pid constants (for pid tuning)

        goal = new State(setpoint.get(), 0);        // creates a new goal for the elevator using the setpoint
        calculated = setPointCalc.calculate(loopTime, calculated, goal);    // motion profiling uses the current state and the goal state to calculuate new curr state
        motor.setTargetPosition(calculated.position / rotationToMeters.in(Units.Meters), feedforward.calculate(calculated.velocity));     // actually sets the PID and Feedfoward
    }

    /**
     * @return Returns the position of the motor in rotations
     */
    public double getMotorPos() {
        return motor.getPosition();
    };

    /**
     * @return Returns the elevator position in meters
     */
    public double getElevatorPos() {
        return getMotorPos() * rotationToMeters.in(Units.Meters);
    }

    /**
     * Returns the elevators velocity in meters/second
     * @return
     */
    public double getElevatorVelo() {
        return (motor.getVelocity() * rotationToMeters.in(Units.Meters)) / 60.0;
    }

    /**
     * Moves the elevator to your desired position
     * @param setPoint the desired position in meters
     * @param error the amount of acceptable error in meters
     * @return
     */
    public Command moveElevator(double setPoint, double error) {
        return run(() -> setpoint.set(setPoint)).
        until(() -> motor.atPositionSetpoint(setPoint / rotationToMeters.in(Units.Meters), 
                                            error / rotationToMeters.in(Units.Meters)));
    }

    /**
     * This is really not recommended. 
     * Please try to talk your drive team out of doing this.
     * However, if they really want "manual" elevator control, here it is.
     * <p></p>
     * Moves the elevator up 
     * @param adjustment the number of meters to move it up
     * @return
     */
    public Command moveUp(double adjustment) {
        return moveElevator(Math.min(getElevatorPos() + adjustment, maxExtension), 0.1 * adjustment);
    }

    /**
     * This is really not recommended. 
     * Please try to talk your drive team out of doing this.
     * However, if they really want "manual" elevator control, here it is.
     * <p></p>
     * Moves the elevator down
     * @param adjustment the number of meters to move it down
     * @return
     */
    public Command moveDown(double adjustment) {
        return moveElevator(Math.max(getElevatorPos() - adjustment, 0), 0.1 * adjustment);
    }

    /**
     * Sets the motors voltage
     * @param voltage the voltage to be set
     */
    public void runSysIdRoutine(Voltage voltage) {
        motor.setVoltage(voltage.in(Units.Volts));
    }

    /**
     * Logs the motors behavior.
     * @param log
     */
    public void sysIdLogMotors(SysIdRoutineLog log) {
        log.motor("Elevator Motor").
        linearPosition(Units.Meters.of(getElevatorPos())).
        linearVelocity(Units.MetersPerSecond.of(getElevatorVelo())).
        voltage(motor.getVoltage());
    } 
}
