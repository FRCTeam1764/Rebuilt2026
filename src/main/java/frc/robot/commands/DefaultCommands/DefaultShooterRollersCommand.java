// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DefaultCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.CommandConstants;
import frc.robot.subsystems.ShooterRollers;
import frc.robot.subsystems.StateManager;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DefaultShooterRollersCommand extends Command {
  /** Creates a new DefaultShooterRollersCommand. */
  ShooterRollers shooter;
  StateManager stateManager;
  public DefaultShooterRollersCommand(ShooterRollers shooter, StateManager stateManager) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.stateManager = stateManager;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(stateManager.getDesiredData(CommandConstants.SHOOTER_ROLLER_KEY) != null) {
      shooter.wheelsShooter((double) stateManager.getDesiredData(CommandConstants.SHOOTER_ROLLER_KEY));
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
