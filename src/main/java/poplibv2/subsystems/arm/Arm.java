package poplibv2.subsystems.arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import poplib.subsytems.pivot.Pivot;
import poplibv2.motors.MotorConfig;
import poplibv2.control_systems.PIDConfig;
import poplibv2.control_systems.PIDTuning;
import poplibv2.misc.TunableNumber;
import poplibv2.motors.FollowerConfig;
import poplibv2.motors.Motor;

public class Arm extends Pivot {
    private ArmFeedforward feedforward;
    private TrapezoidProfile setPointCalc;
    private TrapezoidProfile.State goal;
    private TrapezoidProfile.State calculated;
    private Angle rotationToDegrees;
    private final double loopTime = 0.02;

    public Arm(MotorConfig leadMotorConfig, FollowerConfig[] followerConfigs, ArmFeedforward feedforward, Angle rotationToDegrees, 
    double maxAngularVelocity, double maxAngularAcceleration, double maxRotation, boolean tuningMode, String subsystemName) {
        super(subsystemName);

        this.motor = new Motor(leadMotorConfig);
        for (FollowerConfig followerConfig : followerConfigs) {
            this.motor.addFollowerMotor(followerConfig);
        }
        this.setpoint = new TunableNumber("Elevator Setpoint", 0, tuningMode);
        this.tuning = new PIDTuning("Arm", new PIDConfig(), tuningMode);
        this.feedforward = feedforward;
        this.setPointCalc = new TrapezoidProfile(new TrapezoidProfile.Constraints(maxAngularVelocity, maxAngularAcceleration));
        this.goal = new TrapezoidProfile.State();
        this.calculated = new TrapezoidProfile.State();
        this.rotationToDegrees = rotationToDegrees;
        this.maxRotation = maxRotation;
    }

    @Override
    public void periodic() {
        motor.changePID(tuning.generatePIDConfig());

        goal = new TrapezoidProfile.State(setpoint.get(), 0);
        calculated = setPointCalc.calculate(loopTime, calculated, goal);
        motor.setTargetPosition(calculated.position, feedforward.calculate(calculated.position, calculated.velocity));
    }

}
