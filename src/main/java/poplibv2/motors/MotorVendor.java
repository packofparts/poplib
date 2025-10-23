package poplibv2.motors;

/**
 * Defines the different Motor Vendors that we work with. Needed for MotorConfig to let it know which motor type to create.
 * Format is COMPANY_NAME_API_NAME
 */
public enum MotorVendor {
    REV_ROBOTICS_SPARK_MAX,
    CTRE_TALON_FX
}
