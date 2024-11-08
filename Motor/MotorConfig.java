package POPLib.Motor;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import POPLib.Control.PIDConfig;
import POPLib.SmartDashboard.PIDTuning;


public class MotorConfig {
     public final PIDConfig pid;
     public final int canId;
     public final String canBus;
     public final int currentLimit;
     public final boolean inversion;
     public final Mode mode;

     private final static String DEFUALT_CANBUS = "rio";

     public MotorConfig(int canId, String canBus, int currentLimit, Boolean inversion, PIDConfig pid, Mode mode) {
          this.canId = canId;
          this.canBus = canBus;
          this.currentLimit = currentLimit;
          this.inversion = inversion;
          this.pid = pid;
          this.mode = mode;
     }

     public MotorConfig(int canId) { this(canId, MotorConfig.DEFUALT_CANBUS, -1, false, PIDConfig.getZeroPid(), Mode.COAST);}

     public MotorConfig(int canId, String canBus) { this(canId, canBus, -1, false, PIDConfig.getZeroPid(), Mode.COAST);}

     public MotorConfig(int canId, String canBus, int currentLimit, Boolean inversion, Mode mode) { this(canId, canBus, currentLimit, inversion, PIDConfig.getZeroPid(), mode);}
  
     public MotorConfig(int canId, int currentLimit, Boolean inversion, PIDConfig pid, Mode mode) { this(canId, MotorConfig.DEFUALT_CANBUS, currentLimit, inversion, pid, mode); }

     public MotorConfig(int canId, int currentLimit, Boolean inversion, Mode mode) { this(canId, MotorConfig.DEFUALT_CANBUS, currentLimit, inversion, PIDConfig.getZeroPid(), mode);}


     // No CAN Id Motor, used for configs that apply to multiple motors 
     public MotorConfig(String canBus, int currentLimit, Boolean inversion, PIDConfig pid, Mode mode) { this(-1, canBus, currentLimit, inversion, pid, mode); }
     public MotorConfig(int currentLimit, Boolean inversion, PIDConfig pid, Mode mode) { this(-1, MotorConfig.DEFUALT_CANBUS, currentLimit, inversion, pid, mode); }

     public PIDTuning genPIDTuning(String motorName, boolean tuningMode) {
          return new PIDTuning(motorName, pid, tuningMode);
     }

     public TalonFXConfiguration getTalonConfig() {
          TalonFXConfiguration talonConfig = new TalonFXConfiguration();

          MotorHelper.updateSupplyCurrentLimit(currentLimit, talonConfig);

          talonConfig.MotorOutput.NeutralMode = mode.getTalonMode();
          talonConfig.MotorOutput.Inverted = inversion ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;

          pid.updatePidConfig(talonConfig);

          return talonConfig;
     }

     public TalonFX createTalon() {
          TalonFX motor = new TalonFX(canId, canBus);
          motor.getConfigurator().apply(getTalonConfig());
          return motor;
     }

     public void setCanSparkMaxConfig(CANSparkMax motor, CANSparkLowLevel.MotorType type) {
          motor.restoreFactoryDefaults();
          motor.setInverted(inversion);

          motor.setSmartCurrentLimit(currentLimit);

          motor.setIdleMode(mode.getSparkMaxMode());

          if (type == CANSparkLowLevel.MotorType.kBrushless) {
               motor.getEncoder().setPosition(0);
          }

          pid.setPid(motor);
     }

     public MotorConfig getInvertedConfig() { return new MotorConfig(canId, canBus, currentLimit, !inversion, pid, mode); }

     public CANSparkMax createSparkMax() { return createSparkMax(CANSparkLowLevel.MotorType.kBrushless); }

     public CANSparkMax createSparkMax(CANSparkLowLevel.MotorType type) {
          CANSparkMax motor = new CANSparkMax(canId, type);
          setCanSparkMaxConfig(motor, type);
          return motor;
     }
}
