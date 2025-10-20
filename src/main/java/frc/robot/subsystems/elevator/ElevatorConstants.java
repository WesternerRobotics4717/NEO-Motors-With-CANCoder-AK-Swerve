package frc.robot.subsystems.elevator;

public class ElevatorConstants {
    public static final int elevatorID = 44;

    // Profile Constraints
    public static final double kMaxVelocity = 800; // in units per second
    public static final double kMaxAcceleration = 800; // in units per second^2
    
    public static final double kMaxPosition = 6.282;
    public static final double kMinPosition = -0.6;

    // PID Values
    public static final double kP = 1.25;
    public static final double kI = 0.05;
    public static final double kD = 0.0;

    // FF Values
    public static final double kG = 0.2;
    public static final double kS = 0.0;
    public static final double kV = 0.0;
    public static final double kA = 0.0;
    
}
