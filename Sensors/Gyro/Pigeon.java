package POPLib.Sensors.Gyro;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;

/**
 * Encapsulates pigeon gyroscope.
 */
public class Pigeon extends Gyro {
    private final Pigeon2 gyro;
    private final boolean inversion;

    /**
     * Set id and inversion of pigeon.
     */
    public Pigeon(int id, boolean inversion) {
        this(id, inversion, "");
    }

    /**
     * Set id, inversion, and canivore name of pigeon.
     */
    public Pigeon(int id, boolean inversion, String canBusName) {
        gyro = new Pigeon2(id, canBusName);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        this.inversion = inversion;
    }

    @Override
    public Rotation2d getAngle() {
        Rotation2d yaw = getYaw();


        return inversion 
            ? Rotation2d.fromDegrees(MathUtil.inputModulus(360 - yaw.getDegrees(), 0, 360))
            : Rotation2d.fromDegrees(MathUtil.inputModulus(yaw.getDegrees(), 0, 360));
    }

    public Rotation2d getYaw() {
        return new Rotation2d(gyro.getYaw().getValue());
    }

    public Rotation2d getPitch() {
        return new Rotation2d(gyro.getPitch().getValue());
    }

    public Rotation2d getRoll() {
        return new Rotation2d(gyro.getRoll().getValue());
    }

    @Override
    public void zeroGyro() {
        gyro.setYaw(0);
    }

    @Override
    public void setAngle(Rotation2d newAngle) {
        gyro.setYaw(newAngle.getDegrees()); 
    }

    @Override
    public Rotation2d getAngularVelo() {
        return Rotation2d.fromDegrees(gyro.getAngularVelocityZDevice().getValue().in(Units.RotationsPerSecond));
    }

    @Override
    public Rotation2d getLatencyCompensatedAngle() {
        Measure<AngleUnit> yaw = BaseStatusSignal.getLatencyCompensatedValue(gyro.getYaw().refresh(), gyro.getAngularVelocityZDevice().refresh());

        return inversion 
            ? Rotation2d.fromDegrees(MathUtil.inputModulus(360 - yaw.in(Units.Degree), 0, 360))
            : Rotation2d.fromDegrees(MathUtil.inputModulus(yaw.in(Units.Degrees), 0, 360));
    }

}
