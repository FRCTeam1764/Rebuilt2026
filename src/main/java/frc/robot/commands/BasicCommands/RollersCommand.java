// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BasicCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.CommandConstants;
import frc.robot.subsystems.RollersSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RollersCommand extends Command {
  /** Creates a new SpindexerCommand. */
  double intakeRollerSpeed;
  double indexRollerSpeed;
  double spindexerSpeed;
  double flywheelSpeed;
  boolean useIntake;
  RollersSubsystem rollersSubsystem;
  public RollersCommand(RollersSubsystem rollers, double intakeRollerSpeed, double indexRollerSpeed, double spindexerSpeed, double flywheelSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeRollerSpeed = intakeRollerSpeed;
    this.indexRollerSpeed = indexRollerSpeed;
    this.spindexerSpeed = spindexerSpeed;
    this.flywheelSpeed = flywheelSpeed;
    rollersSubsystem = rollers;
  }

  public RollersCommand(RollersSubsystem rollers, boolean withIntake, double intakeSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    if (withIntake) {
      indexRollerSpeed = CommandConstants.INDEX_SPEED;
      spindexerSpeed = CommandConstants.SPINDEXER_SPEED;
      flywheelSpeed = CommandConstants.SHOOTER_SPEED;
      intakeRollerSpeed = intakeSpeed;
      useIntake = true;
    } else {
      indexRollerSpeed = CommandConstants.INDEX_SPEED;
      spindexerSpeed = CommandConstants.SPINDEXER_SPEED;
      flywheelSpeed = CommandConstants.SHOOTER_SPEED;
      useIntake = false;
    }
    rollersSubsystem = rollers;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (useIntake) {
      rollersSubsystem.rollersOn(indexRollerSpeed, flywheelSpeed, spindexerSpeed, intakeRollerSpeed);
    } else {
      rollersSubsystem.indexOn(indexRollerSpeed);
      rollersSubsystem.flywheelsOn(flywheelSpeed);
      rollersSubsystem.spindexOn(spindexerSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (useIntake) {
      rollersSubsystem.rollersOn(0, 0, 0, 0);
    } else {
      rollersSubsystem.indexOn(0);
      rollersSubsystem.flywheelsOn(0);
      rollersSubsystem.spindexOn(0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
