package frc.robot.constants;

import frc.robot.common.CanPort;

public final class Constants {
    
    public static final String CANIVORE_NAME = "1764 canivore";

    public static final CanPort WRIST_MOTOR1 = new CanPort(54);

    public static final CanPort INTAKE_CANCODER = new CanPort(0);

    public static final int INTAKE_LIMITSWITCH = 7;
     
    //ids
    public static final CanPort TURRET_MOTOR = new CanPort(1);
    public static final CanPort WRIST_MOTOR = new CanPort(2);
    public static final CanPort SHOOTER_ROLLER_MOTOR = new CanPort(2);
    public static final CanPort INTAKE_MOTOR = new CanPort(3);
    public static final CanPort INDEX_MOTOR = new CanPort(4);
    
}

