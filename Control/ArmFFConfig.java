package POPLib.Control;

import edu.wpi.first.math.controller.ArmFeedforward;

public class ArmFFConfig {
    public final double G;
    public final double S;
    public final double V;

    public ArmFFConfig(double G, double S, double V) {
        this.G = G;
        this.S = S;
        this.V = V;
    }

    public ArmFFConfig(double G) { this(G, 0.0, 0.0); }

    public ArmFeedforward getArmFeedforward() {
        return new ArmFeedforward(S, G, V);
    }
}
