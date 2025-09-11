package poplibv2.subsystems.swerve.setup;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.LinearVelocity;

public enum SwerveModuleType {
    MK4iL3(
        (150.0 / 7.0), 
        6.12, 
        Units.MetersPerSecond.of(5.6)
    ),
    MK4iL2FOC(
        (150.0 / 7.0), 
        6.75, 
        Units.MetersPerSecond.of(4.572)
    ),
    MK4(
        12.8, 
        6.12, 
        Units.MetersPerSecond.of(5.21208)
    );

    public double angleGearRatio;
    public double driveGearRatio;
    public LinearVelocity maxSpeed;

    private SwerveModuleType(double angleGearRatio, double driveGearRatio, LinearVelocity maxSpeed) {
        this.angleGearRatio = angleGearRatio;
        this.driveGearRatio = driveGearRatio;
        this.maxSpeed = maxSpeed;
    }
}