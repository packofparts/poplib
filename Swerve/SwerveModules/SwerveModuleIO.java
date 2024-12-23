package POPLib.Swerve.SwerveModules;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;
import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;


public interface SwerveModuleIO {
    @AutoLog
    public static class ModuleIOInputs {

      public Rotation2d moduleRotation = new Rotation2d();
      public SwerveModuleState swerveState = new SwerveModuleState();
      public SwerveModulePosition swervePose = new SwerveModulePosition();

      public Distance driveMotorDistance = Units.Meters.zero();
      public LinearVelocity driveVelocityMetersPerSec = Units.MetersPerSecond.zero();
      public Current driveCurrentAmps = Units.Amps.zero();
      public Voltage driveAppliedVolts = Units.Volts.zero();
  
      public Rotation2d turnEncoderPosition = new Rotation2d();
      public Rotation2d turnMotorPosition = new Rotation2d();
      public Current turnCurrentAmps = Units.Amps.zero();
      public Voltage turnAppliedVolts = Units.Volts.zero();
  
      public double[] odometryTimestamps = new double[] {};
      public double[] odometryDrivePositionsRad = new double[] {};
      public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
    }
    
    /** Updates the set of loggable inputs. */
    public default void updateInputs(ModuleIOInputsAutoLogged inputs) {}


}
