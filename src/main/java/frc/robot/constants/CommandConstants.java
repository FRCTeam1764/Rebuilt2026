package frc.robot.constants;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import frc.robot.generated.TunerConstants;

public final class CommandConstants {
    //for talon fx, 0.916 inches per rotation\\

    //limits
    public final static double INTAKE_LIMIT_DOWN = 0.993;
    public final static double INTAKE_LIMIT_UP = 0.745;

    public final static double SHOOTER_LIMIT_UP = 0.6;
    public final static double SHOOTER_LIMIT_DOWN = 0.4;

    public final static double CLIMBER_LIMIT_UP = 0.0;
    public final static double CLIMBER_LIMIT_DOWN = 0.0;

    //positions
    public final static double INTAKE_WRIST_DOWN = 0.988;
    public final static double INTAKE_WRIST_MID = 0.85;
    public final static double INTAKE_WRIST_IN = 0.75;

    public final static double SHOOTER_DEFAULT = 0.5;
    public final static double SHOOTER_WRIST_DOWN = 0.9;
    public final static double SHOOTER_WRIST_UP = 0.5;

    //speeds
    public final static double INTAKE_IN_SPEED = -0.325;
    public final static double INTAKE_OUT_SPEED = 0.1;
    public final static double INDEX_SPEED = -0.25;
    public final static double SHOOTER_SPEED = 1.0;
    public final static double RES_SPEED = 0.35;
    public final static double TURRET_SPEED = 0.3;

    //keys
    public static final String SHOOTER_FLYWHEEL_KEY = "ShooterRollerSpeed";
    public static final String INTAKE_KEY = "IntakeRollerSpeed";
    public static final String RES_INDEX_KEY = "ResevoirRollerSpeed";
    public static final String INTAKE_WRIST_KEY = "IntakeWristSpeed";
    public static final String INDEX_KEY = "IndexRollerSpeed";
    public static final String CLIMBER_KEY = "ClimberPosition";

    //swerve speed
    public static final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public static final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    //pid
    public static final double kP = 4;
    public static final double kI = 0;
    public static final double kD = 0.1; 
    public static final double kTurnToleranceRad = 0.05;
    public static final double kTurnRateToleranceRadPerS = 0.25;

    public static final double drivekP = .1;
    public static final double driveKi = 0;
    public static final double drivekD = 0;

    
        
}