// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BasicCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.CommandConstants;
import frc.robot.subsystems.RollersSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShooterFlywheelCommand extends Command {
  /** Creates a new ShooterFlywheelCommand. */
  double desired;
  RollersSubsystem rollersSubsystem;
  public ShooterFlywheelCommand(RollersSubsystem rollers, double desired) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.desired = desired;
    rollersSubsystem = rollers;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rollersSubsystem.flywheelsOn(CommandConstants.SHOOTER_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
