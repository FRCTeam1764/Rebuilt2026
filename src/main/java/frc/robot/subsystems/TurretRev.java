// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import java.util.EnumSet;

// import com.ctre.phoenix6.configs.CANcoderConfiguration;
// import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
// import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
// import com.revrobotics.PersistMode;
// import com.revrobotics.ResetMode;
// import com.revrobotics.spark.ClosedLoopSlot;
// import com.revrobotics.spark.FeedbackSensor;
// import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkSoftLimit;
// import com.revrobotics.spark.config.ClosedLoopConfig;
// import com.revrobotics.spark.config.SoftLimitConfig;
// import com.revrobotics.spark.config.SparkBaseConfig;
// import com.revrobotics.spark.config.SparkMaxConfig;

// import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
// import edu.wpi.first.networktables.NetworkTableEvent.Kind;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableListener;
// import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
// import edu.wpi.first.wpilibj.event.NetworkBooleanEvent;
// import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CommandConstants;
// import frc.robot.constants.Constants;


public class TurretRev extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  private final SparkMax turretMotor = new SparkMax(23, SparkLowLevel.MotorType.kBrushless);
  // private final SparkClosedLoopController turretPID = turretMotor.getClosedLoopController();
  SparkMaxConfig turretConfig = new SparkMaxConfig();
  DigitalOutput limitSwitch = new DigitalOutput(2);
  CANcoder turretEncoder = new CANcoder(4);
  boolean left = false;
  boolean switchSides = false;
  boolean pressed = false;
  double cheaterEncoder;
  boolean enforceLimits = true;

  double turretLeftMax =  0.40;// 0.46; //-1.55 // FIXME actual encoder val - not ratio adjusted yet
  double turretRightMax = -0.26;//-0.32; //0.75  // FIXME actual encoder val - not ratio adjusted yet

  PIDController pid1 = new PIDController(3.7, 0, 0); // kp was: 1.7
  PIDController pid2 = new PIDController(4.5, 0, 0); // kp was: 2.5

  double calculation;

  double turretP = 0.5;
  
  public TurretRev() {
    SmartDashboard.putBoolean("Turret Limits Enabled", enforceLimits);
    SmartDashboard.putNumber("Turret Voltage Set", 0);

    SmartDashboard.putNumber("Turret P", turretP);
    turretConfig.closedLoop.p(turretP);
    turretConfig.idleMode(IdleMode.kBrake);
    turretMotor.configure(turretConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);

    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    NetworkTable sd = nt.getTable("SmartDashboard");
    DoubleSubscriber turretPSub = sd.getDoubleTopic("Turret P").subscribe(turretP);

    NetworkTableListener.createListener(turretPSub, EnumSet.of(NetworkTableEvent.Kind.kValueAll),
      event -> {
        turretP = event.valueData.value.getDouble();
        turretConfig.closedLoop.p(turretP);
        turretMotor.configure(turretConfig,ResetMode.kResetSafeParameters,PersistMode.kNoPersistParameters);
      }
    );
  }

  public void onPosition(double desiredPos) { 
    turretMotor.getClosedLoopController().setSetpoint(desiredPos, ControlType.kPosition);
  }

  public void onPositionEx(double desiredPos) {
    if ((getPos() > 300 && calculation>0) || (getPos() < 10 && calculation<0)) {
      turretMotor.set(0);
    } else {
      calculation = pid1.calculate(getPos(), desiredPos);
      SmartDashboard.putNumber("Turret onPositionEx calc speed", calculation);
      turretMotor.set(calculation);
    }
  }

  public void onAngleEx(double desiredPos) {
    if ((getAngle() > 300 && calculation>0) || (getAngle() < 10 && calculation<0)) {
      turretMotor.set(0);
    } else {
      calculation = pid1.calculate(getAngle(), desiredPos);
      SmartDashboard.putNumber("Turret onAngleEx calc speed", calculation);
      turretMotor.set(calculation);
    }
  }

  public void onAnglePosition(double desiredAngle) { 
    turretMotor.getClosedLoopController().setSetpoint(desiredAngle/360, ControlType.kPosition);
  }
  
  // 0.3658 is straight forward as of 2026-04-08
  // public void aimStraight() {
  //   turretMotor.set(0.3658);
  // }


  // gear ratio between motor and turret was: 2.41:1
  // will be - theoretically: 2.5 -> 2.41/2.5 = 1:0.964 = 1/0.964:1 = 1.037:1
  static final double encoderRotationRatio = 1.037;
  // TODO find offset (in encoder clicks) from turret forward to encoder forward
  static final double turretOffset = 0.0;
  public double getPos() {
    return turretEncoder.getAbsolutePosition().getValueAsDouble() * encoderRotationRatio + turretOffset;
  }

  public double getAngle() {
    return getPos() * 360;
  }

  public void onSpeed(double speed) {
    if(enforceLimits) {
      if ((speed < 0 && getPos() < turretLeftMax) || (speed > 0 && getPos() > turretRightMax)) {
        turretMotor.set(speed > 0.2 ? 0.2 : speed < -0.2 ? -0.2 : speed*.3); // was: *.25
      } else {
        turretMotor.set(0);
      }
    } else {
      turretMotor.set(speed*.05);
    }
  }

  public void setVoltage(double volts) {
    turretMotor.setVoltage(volts);
    SmartDashboard.putNumber("Turret Voltage Set", volts);
  }

  public void on(boolean neg) {
    turretMotor.set(neg ? -CommandConstants.TURRET_SPEED : CommandConstants.TURRET_SPEED);
  }

  public void stop() {
    turretMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("TurretPosition", turretEncoder.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber("TurretTemperature", turretMotor.getMotorTemperature());
    SmartDashboard.putNumber("TurretCurrent", turretMotor.getOutputCurrent());
    SmartDashboard.putNumber("TurretVoltage", turretMotor.getAppliedOutput());
    SmartDashboard.putNumber("TurretSetpoint", turretMotor.getClosedLoopController().getSetpoint());
    // SmartDashboard.putNumber("TurretSpeed", turretEncoder.getVelocity().getValueAsDouble());
    SmartDashboard.putString("TurretSpeed", String.format("%.2f", turretEncoder.getVelocity().getValueAsDouble()));
    enforceLimits = SmartDashboard.getBoolean("Turret Limits Enabled", true);
    SmartDashboard.putBoolean("Turret Limits Enabled system", enforceLimits);

    // SmartDashboard.putBoolean("Turret Limit Switch", limitSwitch.get());
    // SmartDashboard.putBoolean("turret pressed", pressed);
    //
    // if (limitSwitch.get() && !pressed) {
    //   turretEncoder.setPosition(0);
    //   pressed = true;
    // } else if (!limitSwitch.get() && pressed) {
    //   pressed = false;
    // }
  }
}