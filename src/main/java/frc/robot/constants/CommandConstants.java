package frc.robot.constants;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import frc.robot.generated.TunerConstants;

public final class CommandConstants {
    //for talon fx, 0.916 inches per rotation\\

    //limits
    public final static double INTAKE_LIMIT_DOWN = 0.9;
    public final static double INTAKE_LIMIT_UP = 0.2;

    public final static double SHOOTER_LIMIT_UP = 0.6;
    public final static double SHOOTER_LIMIT_DOWN = 0.4;

    public final static double CLIMBER_LIMIT_UP = 0.0;
    public final static double CLIMBER_LIMIT_DOWN = 0.0;

    //positions
    public final static double INTAKE_WRIST_OVEREXTEND = 0.835;
    public final static double INTAKE_WRIST_DOWN = 0.835;
    public final static double INTAKE_WRIST_MID = 0.11;
    public final static double INTAKE_WRIST_MID_MANUAL = 0.4;
    public final static double INTAKE_WRIST_IN = 0.11;
    public final static double INTAKE_WRIST_IN_IN = 0.11;

    public final static double SHOOTER_WRIST_DOWN = 0.9;
    public final static double SHOOTER_WRIST_UP = 0.6;

    public final static double SHOOTER_FAR = 0.48;
    public final static double SHOOTER_MID1 = 0.44;
    public final static double SHOOTER_MID2 = 0.42;
    public final static double SHOOTER_CLOSE = 0.6;

    public final static double R1_SHOOTER = 0.74; //0.24;
    public final static double R2_SHOOTER = 0.77; //0.27;
    public final static double R3_SHOOTER = 0.80; //0.30;
    public final static double R4_SHOOTER = 0.88; //0.38;
    public final static double RDOWN_SHOOTER = 0.89; //0.39;

    //speeds//

    //intaking in intake roller
    public final static double INTAKE_IN_SPEED = -0.4; //used to -0.7

    //shoot with intake out speed
    public final static double INTAKE_OUT_SPEED = 0.1; //used to 0.1

    //indexer speed
    public final static double INDEX_SPEED = 0.3; //used to 0.6

    //shooter flywheel speed
    public final static double SHOOTER_SPEED = 0.7; //used to 1.0

    //spindexer speed
    public final static double SPINDEXER_SPEED = 0.6; // used to 0.6


    //dont use
    public final static double SHOOTER_SLOW_SPEED = 0.35;
    public final static double TURRET_SPEED = 0.3;
    public final static double CLIMBER_SPEED_UP = 0.7;
    public final static double CLIMBER_SPEED_DOWN = 0.3;

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