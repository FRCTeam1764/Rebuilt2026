// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DefaultCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.CommandConstants;
import frc.robot.subsystems.RollersSubsystem;
import frc.robot.subsystems.StateManager;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DefaultRollersCommand extends Command {
  /** Creates a new DefaultRollersCommand. */
  RollersSubsystem rollers;
  StateManager stateManager;
  public DefaultRollersCommand(RollersSubsystem rollers, StateManager stateManager) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.rollers = rollers;
    this.stateManager = stateManager;
    addRequirements(rollers);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(stateManager.getDesiredData(CommandConstants.INDEX_KEY) != null) {
      rollers.indexOn((double) stateManager.getDesiredData(CommandConstants.INDEX_KEY));
    }
    if(stateManager.getDesiredData(CommandConstants.INTAKE_KEY) != null) {
      rollers.intakeOn((double) stateManager.getDesiredData(CommandConstants.INTAKE_KEY));
    }
    if(stateManager.getDesiredData(CommandConstants.SHOOTER_FLYWHEEL_KEY) != null) {
      rollers.flywheelsOn((double) stateManager.getDesiredData(CommandConstants.SHOOTER_FLYWHEEL_KEY));
    }
    if(stateManager.getDesiredData(CommandConstants.RES_INDEX_KEY) != null) {
      rollers.resOn((double) stateManager.getDesiredData(CommandConstants.RES_INDEX_KEY));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
