// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterWrist extends SubsystemBase {
  /** Creates a new ShooterWrist. */
  public ShooterWrist() {}

  TalonFX wristMotor = new TalonFX(27);
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private double calculation;
  PIDController controller = new PIDController(,, );
// LOOK INTO: above line is missing values becasue I am not sure what values to insert, nor do i just want to copy previous values

  public void flex(double angle){
    if (wristMotor.getPosition().getValueAsDouble() > 300 || wristMotor.getPosition().getValueAsDouble() < 10){
      wristMotor.set(0);
    }
    else {
      calculation = controller.calculate(wristMotor.getPosition(), angle)
      SmartDashboard.putNumber("SHOOTER_WRIST_PID", angle);
      wristMotor.set(calculation);
      // LOOK INTO: i don't know what to do abt calculate error or if my string is correct for smartdashboard, come back to it later
    }
  }

  public void startFlex(){
    wristMotor.set(.1); // this value is just a filler
  }

  public void stopFlex(){
    wristMotor.set(0); 
  }
}
