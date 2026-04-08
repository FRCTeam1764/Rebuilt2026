// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.hal.PowerDistributionVersion;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkSoftLimit;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import static edu.wpi.first.units.Units.Volts;

public class IntakeWristRev extends SubsystemBase {
  /** Creates a new IntakeWrist. */

  SparkMax wristMotor = new SparkMax(62, SparkLowLevel.MotorType.kBrushless);

  public IntakeWristRev() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.inverted(true);
    config.idleMode(SparkBaseConfig.IdleMode.kBrake);

    ClosedLoopConfig pidConfig = new ClosedLoopConfig();
    pidConfig.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    pidConfig.pid(1.5, 0, 0.9, ClosedLoopSlot.kSlot0); //1,225
    pidConfig.pid(0.5, 0, 0.9, ClosedLoopSlot.kSlot1);
    pidConfig.outputRange(-0.3, 0.4);
    pidConfig.allowedClosedLoopError(0.005, ClosedLoopSlot.kSlot0); 
    pidConfig.allowedClosedLoopError(0.005, ClosedLoopSlot.kSlot1); 
    
    FeedForwardConfig ffConfig = new FeedForwardConfig();
    ffConfig.svag(0, 0.2, 0, 1.1);

    pidConfig.apply(ffConfig);
    config.apply(pidConfig);

    wristMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void stop() {
    wristMotor.set(0);
  } 

  public void setPos(double desiredPos) {
    wristMotor.getClosedLoopController().setSetpoint(desiredPos, ControlType.kPosition);
    SmartDashboard.putNumber("intake pid controller", wristMotor.getClosedLoopController().getSetpoint());
  }

  public double getPos() {
    return wristMotor.getAbsoluteEncoder().getPosition();
  }

  public boolean atPos() {
    return wristMotor.getClosedLoopController().isAtSetpoint();
  }

  public double getSpeed() {
    return wristMotor.getAbsoluteEncoder().getVelocity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("IntakeWristPosition", wristMotor.getAbsoluteEncoder().getPosition());
    SmartDashboard.putNumber("IntakeWristTemperature", wristMotor.getMotorTemperature());
    SmartDashboard.putNumber("IntakeWristCurrent", wristMotor.getOutputCurrent());
    SmartDashboard.putNumber("IntakeWristSetpoint", wristMotor.getClosedLoopController().getSetpoint());
    SmartDashboard.putNumber("IntakeWristVoltage", wristMotor.getAppliedOutput());
  }
}