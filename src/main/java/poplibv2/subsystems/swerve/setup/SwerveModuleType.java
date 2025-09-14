package poplibv2.subsystems.swerve.setup;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;

/**
 * A enum that describes constants for the different types of swerve modules
 */
public enum SwerveModuleType {
    MK4iL3(
        (150.0 / 7.0), 
        6.12, 
        Units.MetersPerSecond.of(5.6),
        Units.RadiansPerSecond.of(4 * Math.PI)
    ),
    MK4iL2FOC(
        (150.0 / 7.0), 
        6.75, 
        Units.MetersPerSecond.of(4.572),
        Units.RadiansPerSecond.of(4 * Math.PI)
    ),
    MK4(
        12.8, 
        6.12, 
        Units.MetersPerSecond.of(5.21208),
        Units.RadiansPerSecond.of(4 * Math.PI)
    );

    public double angleGearRatio;
    public double driveGearRatio;
    public LinearVelocity maxSpeed;
    public AngularVelocity maxAngularVelocity;

    private SwerveModuleType(double angleGearRatio, double driveGearRatio, LinearVelocity maxSpeed, AngularVelocity maxAngularVelocity) {
        this.angleGearRatio = angleGearRatio;
        this.driveGearRatio = driveGearRatio;
        this.maxSpeed = maxSpeed;
        this.maxAngularVelocity = maxAngularVelocity;
    }
}