package poplibv2.sensors.beam_break;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj.DigitalInput;

public class BeamBreak {
    private final DigitalInput beamBreak;

    public BeamBreak(int portId) {
        this.beamBreak = new DigitalInput(portId);
    }

    /**
     * @return whether or not there is an object infront of the beam break.
     */
    public boolean isBlocked() {
        return !beamBreak.get();
    } 

    /**
     * Creates a BooleanSupplier that returns if the beambreak blocked status matches the parameter boolean value
     * @param value The value to check for matching
     * @return A BooleanSupplier that can be used for command chaining 
     */
    public BooleanSupplier isEqualTo(boolean value) {
        return () -> {
            if (value) {
                return isBlocked();
            }
            return !isBlocked();
        };
    }

    /**
     * Creates a BooleanSupplier that returns if the is beambreak blocked
     * @return A BooleanSupplier that can be used for command chaining 
     */
    public BooleanSupplier getBlockedSupplier() {
        return () -> isBlocked();
    }

    /**
     * Creates a BooleanSupplier that returns if the is beambreak unblocked
     * @return A BooleanSupplier that can be used for command chaining 
     */
    public BooleanSupplier getUnBlockedSupplier() {
        return () -> !isBlocked();
    }
}
