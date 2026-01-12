package frc.robot.constants;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import frc.robot.generated.TunerConstants;

public final class CommandConstants {
    //for talon fx, 0.916 inches per rotation\
    
    //speed
    

    //keys
    public static final String INTAKE_KEY = "WristEncoderPosition";

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