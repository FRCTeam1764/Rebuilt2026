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
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CommandConstants;
import frc.robot.constants.Constants;


public class ShooterWristRev extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  private final SparkMax wristMotor = new SparkMax(Constants.SHOOTER_WRIST_MOTOR.id, SparkLowLevel.MotorType.kBrushless);

  // 's' for start, 'e' for end, 'n' for nothing
  private char reset = 'n';
  
  public ShooterWristRev() {

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
    wristMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void onPosition(double desiredPos) { 
    wristMotor.getClosedLoopController().setSetpoint(desiredPos, ControlType.kPosition);
  }

  public void onAnglePosition(double desiredAngle) { 
    wristMotor.getClosedLoopController().setSetpoint(desiredAngle/360, ControlType.kPosition);
  }

  public double getPos() {
    return wristMotor.getEncoder().getPosition();
  }

  public void onSpeed(double speed) {
    wristMotor.set(speed/3);
  }

  public void on(boolean neg) {
    wristMotor.set(neg ? -CommandConstants.SHOOTER_WRIST_SPEED : CommandConstants.SHOOTER_WRIST_SPEED);
  }

  public void stop() {
    wristMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("WristPosition", wristMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("WristTemperature", wristMotor.getMotorTemperature());
    SmartDashboard.putNumber("WristCurrent", wristMotor.getOutputCurrent());
    SmartDashboard.putNumber("WristSetpoint", wristMotor.getClosedLoopController().getSetpoint());
  }
}