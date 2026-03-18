// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.sim.TalonFXSimState.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkSoftLimit;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CommandConstants;
import frc.robot.constants.Constants;


public class TurretRev extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  private final SparkMax turretMotor = new SparkMax(23, SparkLowLevel.MotorType.kBrushless);
  DigitalOutput limitSwitch = new DigitalOutput(2);
  boolean left = false;
  boolean switchSides = false;
  boolean pressed = false;
  double cheaterEncoder;

  // 's' for start, 'e' for end, 'n' for nothing
  private char reset = 'n';
  
  public TurretRev() {

    SparkMaxConfig config = new SparkMaxConfig();
    config.inverted(false);
    config.idleMode(SparkBaseConfig.IdleMode.kBrake);
    config.smartCurrentLimit(20);

    ClosedLoopConfig pidConfig = new ClosedLoopConfig();
    pidConfig.pid(0, 0, 0);
    pidConfig.outputRange(-0.5, 0.5);
    config.apply(pidConfig);

    SoftLimitConfig limitConfig = new SoftLimitConfig();
    limitConfig.forwardSoftLimitEnabled(true);
    limitConfig.forwardSoftLimit(355);
    limitConfig.reverseSoftLimitEnabled(true);
    limitConfig.reverseSoftLimit(5);
    config.apply(limitConfig);

    //config, resets configs to default, configs persist even after motor is power cycled
    turretMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    cheaterEncoder = turretMotor.getAbsoluteEncoder().getPosition();
  }

  public void onPosition(double desiredPos) { 
    turretMotor.getClosedLoopController().setSetpoint(desiredPos, ControlType.kPosition);
  }

  public void onAnglePosition(double desiredAngle) { 
    turretMotor.getClosedLoopController().setSetpoint(desiredAngle/360, ControlType.kPosition);
  }

  public double getAbsPos() {
    return turretMotor.getEncoder().getPosition();
  }

  public double getPos() {
    return cheaterEncoder;
  }

  public boolean noReset() {
    if (reset == 'n') {
      return true;
    }
    return false;
  }

  public void resetTurret() {
    if (reset == 's') {
      onPosition(5);
    } else if (reset == 'e') {
      onPosition(355);
    }
  }

  public void onSpeed(double speed) {
    // if (noReset()) {
    //   turretMotor.set(speed/3);
    // } else {
    //   resetTurret();
    // }
    if (cheaterEncoder>=0.1 && speed<0) { 
      turretMotor.set(speed);
     } else if (cheaterEncoder<=1.9 && speed>0) {
      turretMotor.set(speed/2);
    } else {
      turretMotor.set(0);
    }
  }

  public void on(boolean neg) {
    if (noReset()) {
      turretMotor.set(neg ? -CommandConstants.TURRET_SPEED : CommandConstants.TURRET_SPEED);
    }
  }

  public void stop() {
    turretMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (getPos()>=355) {
      reset = 's';
    } else if (getPos()<=5 && reset == 'n') {
      reset = 'e';
    } else if ((reset == 's' && getPos()<=5) || (reset == 'e' && getPos()>=355)) {
      reset = 'n';
    }

    if (pressed && !limitSwitch.get()) {
      left = !left;
    }

    if (limitSwitch.get()) {
      pressed = true;
      turretMotor.getEncoder().setPosition(0);
    } else {
      pressed = false;
    }
    
    switchSides = SmartDashboard.getBoolean("Switch Turret Side", false);
    SmartDashboard.putBoolean("Switch Turret Side", switchSides);

    if (switchSides) {
      left = !left;
      switchSides = false;
    }

    SmartDashboard.putBoolean("Left Side Turret", left);


    if (left) {
      cheaterEncoder = turretMotor.getAbsoluteEncoder().getPosition();
    } else {
      cheaterEncoder = turretMotor.getAbsoluteEncoder().getPosition() + 1;
    }

    SmartDashboard.putNumber("TurretPosition", turretMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("TurretTemperature", turretMotor.getMotorTemperature());
    SmartDashboard.putNumber("TurretCurrent", turretMotor.getOutputCurrent());
    SmartDashboard.putString("TurretResetState", String.valueOf(reset));
    SmartDashboard.putNumber("TurretSetpoint", turretMotor.getClosedLoopController().getSetpoint());
    SmartDashboard.putNumber("TurretSpeed", turretMotor.getAbsoluteEncoder().getVelocity());

    SmartDashboard.putNumber("Turret cheater encoder", cheaterEncoder);

    SmartDashboard.putBoolean("Turret Limit Switch", limitSwitch.get());
    SmartDashboard.putBoolean("turret pressed", pressed);
  }
}