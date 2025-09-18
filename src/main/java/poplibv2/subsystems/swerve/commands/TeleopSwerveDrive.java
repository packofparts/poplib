package poplibv2.subsystems.swerve.commands;


import poplibv2.controllers.ControllerMath;
import poplibv2.controllers.io.IO;
import poplibv2.subsystems.swerve.Swerve;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Supplier;

/**
 * Command that controls teleop swerve.
 */
public class TeleopSwerveDrive extends Command {
    private final Swerve swerve;
    private final Supplier<Double> xAxisSupplier;
    private final Supplier<Double> yAxisSupplier;
    private final Supplier<Double> rotSupplier;
    private final double speedMultiplier;
    private final double stickDeadBand;

    /**
     * Creates a command to drive swerve in Teleop.
     * @param swerve The swerve subsystem.
     * @param io The Controller
     * @param speedMultiplier The amount to multiply the inputs by. Use a lower value for "baby mode" (aka when PR wants to drive the robot)
     */
    public TeleopSwerveDrive(Swerve swerve, IO io, double speedMultiplier) {
        this.swerve = swerve;
        this.xAxisSupplier =  io::getDriveTrainTranslationX;
        this.yAxisSupplier = io::getDriveTrainTranslationX;;
        this.rotSupplier = io::getDriveTrainTranslationX;;
        this.speedMultiplier = speedMultiplier;
        this.stickDeadBand = IO.DEADBAND;        
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        double forwardBack = yAxisSupplier.get() * speedMultiplier;
        double leftRight = xAxisSupplier.get() * speedMultiplier;
        double rot = rotSupplier.get() * speedMultiplier;

        forwardBack = ControllerMath.applyDeadband(forwardBack, stickDeadBand);
        leftRight = ControllerMath.applyDeadband(leftRight, stickDeadBand);

        Translation2d translation = new Translation2d(forwardBack, leftRight);

        swerve.drive(
            translation,
            ControllerMath.cube(rot),
            DriverStation.getAlliance().get()
        );
    }
}