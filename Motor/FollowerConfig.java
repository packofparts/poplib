package POPLib.Motor;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

public class FollowerConfig {
    private MotorConfig leadConfig;
    private boolean inverted;
    private int canId;

    public FollowerConfig(MotorConfig leadConfig, boolean inverted, int canId) {
        this.leadConfig = leadConfig;
        this.inverted = inverted;
        this.canId = canId;
    }

    public CANSparkMax createSparkMax(CANSparkMax lead) { 
        CANSparkMax motor = new CANSparkMax(canId, CANSparkLowLevel.MotorType.kBrushless);
        leadConfig.setCanSparkMaxConfig(motor, CANSparkLowLevel.MotorType.kBrushless);
        motor.follow(lead, inverted);

        return motor;
    }

    public TalonFX createTalon() {
        TalonFX motor = new TalonFX(canId);
        motor.getConfigurator().apply(leadConfig.getTalonConfig());
        motor.setControl(new Follower(leadConfig.canId, inverted));
        return motor;
    }
}
