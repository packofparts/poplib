package poplibv2.sensors.camera;

public class LimelightConfig {
    public String limelightName;
    public int pipeline;
    public int ledState;
    public double minValidArea;

    /**
     * Creates a config that can be used to make a new Limelight.
     * @param limelightName The name of the limelight
     * @param pipeline What detection pipeline to use
     * @param ledState What state the green leds should be in
     * @param minValidArea The minimum area a detected object needs to ocupy before being considered as a valid detection.
     */
    public LimelightConfig(String limelightName, int pipeline, int ledState, double minValidArea) {
        this.limelightName = limelightName;
        this.pipeline = pipeline;
        this.ledState = ledState;
        this.minValidArea = minValidArea;
    }
}