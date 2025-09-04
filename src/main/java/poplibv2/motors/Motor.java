package poplibv2.motors;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.Units;
import poplibv2.control_systems.PIDConfig;
import poplibv2.misc.CanIdRegistry;
import poplibv2.misc.ErrorHandling;


public class Motor {

    private SparkMax spark;
    private TalonFX talon;
    private int canID;
    private MotorVendor motorType;
    private SparkMaxConfig sparkConfig;     // needed to change idle mode
    private boolean isConfiguredWithPID;
    private PositionDutyCycle positionDutyCycle;
    private VelocityDutyCycle velocityDutyCycle;

    /**
     * Creates a new motor using the motor config.
     * @param config The config to use when creating the motor.
     */
    public Motor(MotorConfig config) {
        this.canID = config.getCANID();
        CanIdRegistry.getRegistry().registerCanId(canID);
        this.motorType = config.getMotorVendor();

        if (this.motorType == MotorVendor.REV_ROBOTICS_SPARK_MAX) {
            this.spark = new SparkMax(canID, MotorType.kBrushless);
            this.sparkConfig = config.createSparkMaxConfig();
            ErrorHandling.handleRevLibError(
                spark.configure(this.sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters),
                "configuring motor " + canID
            );
            this.talon = null;
            this.motorType = MotorVendor.REV_ROBOTICS_SPARK_MAX;
            this.positionDutyCycle = null;
            this.velocityDutyCycle = null;
        } 
        
        else if (this.motorType == MotorVendor.CTRE_TALON_FX) {
            talon = new TalonFX(canID, config.getCANBUS());
            talon.getConfigurator().apply(config.createTalonFXConfiguration());
            this.spark = null;
            this.sparkConfig = null;
            this.positionDutyCycle = new PositionDutyCycle(0.0).withSlot(talon.getClosedLoopSlot().getValue());
            this.velocityDutyCycle = new VelocityDutyCycle(0.0).withSlot(talon.getClosedLoopSlot().getValue());
        }

        this.isConfiguredWithPID = config.getIsConfiguredWithPID();
    }

    /**
     * Uses a PID Loop to turn the motor to the desired position. (this should be called in periodic)
     * @param position the desired position of the motor. This will already include any calulations done in ConversionConfig 
     * @throws IncorrectUseOfPIDException
     */
    public void setTargetPosition(double position) {
        checkForPID();
        if (motorType == MotorVendor.CTRE_TALON_FX) {
            talon.setControl(positionDutyCycle.withPosition(position));
        } else if (motorType == MotorVendor.REV_ROBOTICS_SPARK_MAX) {
            spark.getClosedLoopController().setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        }
    }

    /**
     * Uses a PID Loop to turn the motor to the desired position. (this should be called in periodic)
     * Also includes whether to use FOC. This will allow your motor to run 15% faster for some trade offs.
     * FOC only works on Kraken Motors (Talon FX), and it requires a paid Pheonix Pro license that should be applied in Pheonix Tuner X. 
     * @param position the desired position of the motor. This will already include any calulations done in ConversionConfig 
     * @param enableFOC whether or not to use FOC.
     * @throws IncorrectUseOfPIDException
     */
    public void setTargetPosition(double position, boolean enableFOC) {
        checkForPID();
        if (motorType == MotorVendor.CTRE_TALON_FX) {
            talon.setControl(positionDutyCycle.withPosition(position).withEnableFOC(enableFOC));
        } else if (motorType == MotorVendor.REV_ROBOTICS_SPARK_MAX) {
            spark.getClosedLoopController().setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        }
    }
    
    /**
     * Uses a PID Loop to turn the motor to the desired position. (this should be called in periodic)
     * Also includes the feedForwardOutput that you want to apply to this motor.
     * @param position the desired position of the motor. This will already include any calulations done in ConversionConfig 
     * @param feedforwardOutput the output you get from doing something like feedforwardController.calculate(velocity)
     * @throws IncorrectUseOfPIDException
     */
    public void setTargetPosition(double position, double feedforwardOutput) {
        checkForPID();
        if (motorType == MotorVendor.CTRE_TALON_FX) {
            talon.setControl(positionDutyCycle.withPosition(position).withFeedForward(feedforwardOutput));
        } else if (motorType == MotorVendor.REV_ROBOTICS_SPARK_MAX) {
            spark.getClosedLoopController().setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0, feedforwardOutput);
        }
    }

    /**
     * Uses a PID Loop to turn the motor to the desired position. (this should be called in periodic)
     * Also includes the feedForwardOutput that you want to apply to this motor.
     * Also includes whether to use FOC. This will allow your motor to run 15% faster for some trade offs.
     * FOC only works on Kraken Motors (Talon FX), and it requires a paid Pheonix Pro license that should be applied in Pheonix Tuner X. 
     * @param position the desired position of the motor. This will already include any calulations done in ConversionConfig 
     * @param feedforwardOutput the output you get from doing something like feedforwardController.calculate(velocity)
     * @param enableFOC whether or not to use FOC.
     * @throws IncorrectUseOfPIDException
     */
    public void setTargetPosition(double position, double feedforwardOutput, boolean enableFOC) {
        checkForPID();
        if (motorType == MotorVendor.CTRE_TALON_FX) {
            talon.setControl(positionDutyCycle.withPosition(position).withFeedForward(feedforwardOutput).withEnableFOC(enableFOC));
        } else if (motorType == MotorVendor.REV_ROBOTICS_SPARK_MAX) {
            spark.getClosedLoopController().setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0, feedforwardOutput);
        }
    }






    /**
     * Uses a PID Loop to run the motor at the desired velocity. (this should be called in periodic)
     * @param velocity the desired velocity of the motor. This will already include any calulations done in ConversionConfig 
     * @throws IncorrectUseOfPIDException
     */
    public void setTargetVelocity(double velocity) {
        checkForPID();
        if (motorType == MotorVendor.CTRE_TALON_FX) {
            talon.setControl(velocityDutyCycle.withVelocity(velocity));
        } else if (motorType == MotorVendor.REV_ROBOTICS_SPARK_MAX) {
            spark.getClosedLoopController().setReference(velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        }
    }

    /**
     * Uses a PID Loop to run the motor at the desired velocity. (this should be called in periodic)
     * Also includes whether to use FOC. This will allow your motor to run 15% faster for some trade offs.
     * FOC only works on Kraken Motors (Talon FX), and it requires a paid Pheonix Pro license that should be applied in Pheonix Tuner X. 
     * @param velocity the desired velocity of the motor. This will already include any calulations done in ConversionConfig 
     * @param enableFOC whether or not to use FOC.
     * @throws IncorrectUseOfPIDException
     */
    public void setTargetVelocity(double velocity, boolean enableFOC) {
        checkForPID();
        if (motorType == MotorVendor.CTRE_TALON_FX) {
            talon.setControl(velocityDutyCycle.withVelocity(velocity).withEnableFOC(enableFOC));
        } else if (motorType == MotorVendor.REV_ROBOTICS_SPARK_MAX) {
            spark.getClosedLoopController().setReference(velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        }
    }

    /**
     * Uses a PID Loop to run the motor at the desired velocity. (this should be called in periodic)
     * Also includes the feedForwardOutput that you want to apply to this motor.
     * @param velocity the desired velocity of the motor. This will already include any calulations done in ConversionConfig 
     * @param feedforwardOutput the output you get from doing something like feedforwardController.calculate(velocity)
     * @throws IncorrectUseOfPIDException
     */
    public void setTargetVelocity(double velocity, double feedForwardOutput) {
        checkForPID();
        if (motorType == MotorVendor.CTRE_TALON_FX) {
            talon.setControl(velocityDutyCycle.withVelocity(velocity).withFeedForward(feedForwardOutput));
        } else if (motorType == MotorVendor.REV_ROBOTICS_SPARK_MAX) {
            spark.getClosedLoopController().setReference(velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0, feedForwardOutput);
        }
    }

    /**
     * Uses a PID Loop to run the motor at the desired velocity. (this should be called in periodic)
     * Also includes the feedForwardOutput that you want to apply to this motor.
     * Also includes whether to use FOC. This will allow your motor to run 15% faster for some trade offs.
     * FOC only works on Kraken Motors (Talon FX), and it requires a paid Pheonix Pro license that should be applied in Pheonix Tuner X. 
     * @param velocity the desired velocity of the motor. This will already include any calulations done in ConversionConfig 
     * @param feedforwardOutput the output you get from doing something like feedforwardController.calculate(velocity)
     * @param enableFOC whether or not to use FOC.
     * @throws IncorrectUseOfPIDException
     */
    public void setTargetVelocity(double velocity, double feedForwardOutput, boolean enableFOC) {
        checkForPID();
        if (motorType == MotorVendor.CTRE_TALON_FX) {
            talon.setControl(velocityDutyCycle.withVelocity(velocity).withFeedForward(feedForwardOutput).withEnableFOC(enableFOC));
        } else if (motorType == MotorVendor.REV_ROBOTICS_SPARK_MAX) {
            spark.getClosedLoopController().setReference(velocity, ControlType.kVelocity, ClosedLoopSlot.kSlot0, feedForwardOutput);
        }
    }





    /**
     * Common interface for setting the speed of a speed controller. IF you have configured this motor to run with PID, DO NOT USE THIS.
     * @param speed The speed to set. Value should be between -1.0 and 1.0.
     */
    public void set(double speed)  {
        if (this.isConfiguredWithPID) {
            ErrorHandling.complainAboutIncorrectUseOfPID(
                "Attempted to use motor.set(speed) on a motor that is configured to use PID. This action will make the motor fight with itself. " + 
                "If you want to make the motor go at a certain speed, tune the PID for velocity and do motor.setTargetVelocity(velocity), which will use the PID loop to run at a certain speed. " +
                "The motor this happened to has the CAN ID: " + this.canID
            );
        } 
        if (motorType == MotorVendor.CTRE_TALON_FX) {
            talon.set(speed);
        } else if (motorType == MotorVendor.REV_ROBOTICS_SPARK_MAX) {
            spark.set(speed);
        }
    }

    /**
     * Returns whether or not the motor has reached its setpoint.
     * @param setpoint the setpoint to use when checking motor position
     * @param error the allowed error
     * @return Whether or not the motor has reached its setpoint within the allowed error.
     */
    public boolean atPositionSetpoint(double setpoint, double error) {
        return Math.abs(setpoint - getPosition()) < error;
    }

    /**
     * Gets the position of the encoder. Note that this position has already been scaled
     * according to the value in the ConversionConfig object that was used to create the MotorConfig object.
     * @return The amount of rotations
     */
    public double getPosition() {
        if (motorType == MotorVendor.CTRE_TALON_FX) {
            return talon.getPosition().getValue().in(Units.Rotations);
        } else if (motorType == MotorVendor.REV_ROBOTICS_SPARK_MAX) {
            return spark.getEncoder().getPosition();
        }
        return 0.0;
    }

    /**
     * Gets the angluar velocity of the motor. Note that this velocity has already been scaled
     * according to the value in the ConversionConfig object that was used to create the MotorConfig object.
     * @return the velocity in RPM
     */
    public double getVelocity() {
        if (motorType == MotorVendor.CTRE_TALON_FX) {
            return talon.getVelocity().getValue().in(Units.RPM);
        } else if (motorType == MotorVendor.REV_ROBOTICS_SPARK_MAX) {
            return spark.getEncoder().getVelocity();
        }
        return 0.0;
    }

    /**
     * Changes the Idle Behavior of the motor (the behavior when the motor is not receiving commands or when the robot is disabled)
     * @param newBehavior the new idle behavior
     */
    public void changeIdleBehavior(IdleBehavior newBehavior) {
        if (motorType == MotorVendor.CTRE_TALON_FX) {
            talon.setNeutralMode(newBehavior == IdleBehavior.BRAKE ? NeutralModeValue.Brake : NeutralModeValue.Coast);
        } else if (motorType == MotorVendor.REV_ROBOTICS_SPARK_MAX) {
            sparkConfig.idleMode(newBehavior == IdleBehavior.BRAKE ? IdleMode.kBrake : IdleMode.kCoast);
            ErrorHandling.handleRevLibError(
                spark.configure(sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters),
                "configuring motor " + canID
            );
        }
    }


    public void changePID(PIDConfig config) {
        checkForPID();
        if (motorType == MotorVendor.CTRE_TALON_FX) {
            Slot0Configs slot0Configs = new Slot0Configs();
            slot0Configs.kV = config.F;
            slot0Configs.kP = config.P;
            slot0Configs.kI = config.I;
            slot0Configs.kD = config.D;
            talon.getConfigurator().apply(slot0Configs);
        } else if (motorType == MotorVendor.REV_ROBOTICS_SPARK_MAX) {
            config.applyToMotor(sparkConfig);
            ErrorHandling.handleRevLibError(
                spark.configure(sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters),
                "configuring motor " + canID
            );
        }
    }

    private void checkForPID() {
        if (!this.isConfiguredWithPID) {
            ErrorHandling.complainAboutIncorrectUseOfPID(
                "Attempted to use PID on a motor that was not configured with PID Constants." +
                "The motor this happened to has the CAN ID: " + this.canID
            );
        }
    }
}
