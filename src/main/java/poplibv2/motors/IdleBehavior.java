package poplibv2.motors;

/**
 * The default behavior of the motor when no control signals are being sent to it. 
 * Brake causes the motor to resist extrenal force and coast allows the motor to freely spin.
 */
public enum IdleBehavior {
    BRAKE,
    COAST
}

