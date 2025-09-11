package poplibv2.sensors.camera;

/**
 * Details what strategy to use when calculating the variable standard deviations for april tag detection 
 */
public enum StdDevStategy {
    DISTANCE,    // recommended strategy
    AMBIGUITY
}
