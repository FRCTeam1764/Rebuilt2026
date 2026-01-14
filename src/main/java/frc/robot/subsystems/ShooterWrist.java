// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class ShooterWrist extends SubsystemBase {
  /** Creates a new ShooterWrist. */

  TalonFX wristMotor = new TalonFX(27);

  private double calculation;
  private VoltageOut voltageOut;
  private StateManager stateManager;
  private PIDController controller = new PIDController(0,0,0);
// PID values, they are NOT right, like VERY WRONG 

  private final SysIdRoutine m_sysIdRoutine = 
    new SysIdRoutine(
      new SysIdRoutine.Config(
        null,
        Volts.of(4),
        null,

        (state) -> SignalLogger.writeString("state", state.toString())
      ),
      new SysIdRoutine.Mechanism(
        (volts) -> wristMotor.setControl(voltageOut.withOutput(volts.in(Volts))),
        null,
        this
      )
    );

    public ShooterWrist(StateManager stateManager) {

      this.stateManager = stateManager;
      TalonFXConfiguration flexConfig = new TalonFXConfiguration();
      flexConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
      flexConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      flexConfig.MotorOutput.PeakForwardDutyCycle = 0.5;
      flexConfig.MotorOutput.PeakReverseDutyCycle = -0.5;
      flexConfig.CurrentLimits.StatorCurrentLimitEnable = true;
      flexConfig.CurrentLimits.StatorCurrentLimit = 60;

      wristMotor.getConfigurator().apply(flexConfig);

    }


  public Command sysIdQuasistatic(SysIdRoutine.Direction direction){
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction){
    return m_sysIdRoutine.dynamic(direction);
  }
  

  public void flex(double angle){
    if (wristMotor.getPosition().getValueAsDouble() > 300 || wristMotor.getPosition().getValueAsDouble() < 10){
      wristMotor.set(0);
    }
    else {
      calculation = controller.calculate(wristMotor.getPosition().getValueAsDouble(), angle);
      SmartDashboard.putNumber("SHOOTER_WRIST_PID", angle);
      wristMotor.set(calculation);
    }
  }

  public void startFlex(){
    wristMotor.set(.1); // this value is just a filler
  }

  public void stopFlex(){
    wristMotor.set(0); 
  }



  @Override
  public void periodic() {
    SmartDashboard.putNumber("ShooterWristPosition",wristMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("ShooterWristTemperature", wristMotor.getDeviceTemp().getValueAsDouble());
    SmartDashboard.putNumber("ShooterWristCurrent", wristMotor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putBoolean("ShooterWristHappy", wristMotor.getStatorCurrent().getValueAsDouble()<30);
    // value 30 in above line is pulled from reefscape intake wrist, might have to change later
    SmartDashboard.putNumber("UnhappyCount", SmartDashboard.getNumber("UnhappyCount", 0)); // reefscape code also included a check for if the elevator was happy
    // This method will be called once per scheduler run

    // more code here in reefscape with stagemanager, prob have to add later
  }
}
