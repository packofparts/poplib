package poplibv2.sensors.gyro;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import poplibv2.misc.CanIdRegistry;


public class Pigeon extends Gyro {
    private final Pigeon2 gyro;
    private final boolean inversion;
    
    /**
     * Creates a new CTRE Pigeon Device (uses the Pigeon2 api)
     * Since this is a CAN Device, the ID is registered in the CAN ID Registry to make sure duplicate IDs are not created.
     * @param id the CAN ID
     * @param inversion Whether or not the electrical subteam placed the gyro the wrong way
     * @param canBusName Pretty self explantory. Default should be "rio" 
     */
    public Pigeon(int id, boolean inversion, String canBusName) {
        CanIdRegistry.getRegistry().registerCanId(id);
        gyro = new Pigeon2(id, canBusName);
        gyro.getConfigurator().apply(new Pigeon2Configuration());   // apperently this actually needs to be done
        this.inversion = inversion;
    }

    @Override
    public Angle getNormalizedAngle() {
        return invertAndNormalizeAngle(getYaw());
    }

    @Override
    public Angle getYaw() {
        return gyro.getYaw().getValue();
    }

    @Override
    public Angle getPitch() {
        return gyro.getPitch().getValue();
    }

    @Override
    public Angle getRoll() {
        return gyro.getRoll().getValue();
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
    public AngularVelocity getAngularVelo() {
        return gyro.getAngularVelocityZDevice().getValue();
    }

    @Override
    public Angle getLatencyCompensatedAngle() {
        Measure<AngleUnit> yaw = BaseStatusSignal.getLatencyCompensatedValue(gyro.getYaw().refresh(), gyro.getAngularVelocityZDevice().refresh());
        return invertAndNormalizeAngle(yaw);
    }

    private Angle invertAndNormalizeAngle(Measure<AngleUnit> angle) {
        return inversion ? Units.Degrees.of(MathUtil.inputModulus(360 - angle.in(Units.Degrees), 0, 360))
                         : Units.Degrees.of(MathUtil.inputModulus(angle.in(Units.Degrees), 0, 360));
    }

    @Override
    public Rotation2d getNormalizedRotation2dAngle() {
        return new Rotation2d(getNormalizedAngle());
    }

}
