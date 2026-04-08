// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RollersSubsystem extends SubsystemBase {
  /** Creates a new rollerSubsystem. */
  TalonFX indexRollers = new TalonFX(8);
  TalonFX shooterFlywheel = new TalonFX(9);
  TalonFX spindexer = new TalonFX(7);
  TalonFX intakeRollers = new TalonFX(6);

  public RollersSubsystem() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = 30;

    indexRollers.getConfigurator().apply(config);
    shooterFlywheel.getConfigurator().apply(config);
    spindexer.getConfigurator().apply(config);
    intakeRollers.getConfigurator().apply(config);

  }

  public void rollersOn(double speedIndex, double speedShooter, double speedRes, double speedIntake) {
    indexRollers.set(speedIndex);
    shooterFlywheel.set(speedShooter);
    spindexer.set(speedRes);
    intakeRollers.set(speedIntake);
    
  }

  public void intakeOn(double speed) {
    intakeRollers.set(speed);
  }

  public void spindexOn(double speed) {
    spindexer.set(speed);
  }

  public void flywheelsOn(double speed) {
    shooterFlywheel.set(speed);
  }

  public void indexOn(double speed) {
    indexRollers.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("index roller voltage", indexRollers.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("index roller current", indexRollers.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("index roller temp", indexRollers.getDeviceTemp().getValueAsDouble());

    SmartDashboard.putNumber("intake roller voltage", intakeRollers.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("intake roller current", intakeRollers.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("intake roller temp", intakeRollers.getDeviceTemp().getValueAsDouble());

    SmartDashboard.putNumber("shooter flywheel voltage", shooterFlywheel.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("shooter flywheel current", shooterFlywheel.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("shooter flywheel temp", shooterFlywheel.getDeviceTemp().getValueAsDouble());

    SmartDashboard.putNumber("res roller voltage", spindexer.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("res roller current", spindexer.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("res roller temp", spindexer.getDeviceTemp().getValueAsDouble());
  }
}
