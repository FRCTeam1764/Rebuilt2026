// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BasicCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.CommandConstants;
import frc.robot.subsystems.Turret;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurretCommand extends Command {
  /** Creates a new TurretCommand. */
  Turret turret;
  int desired;
  public TurretCommand(Turret turret, int desired) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.turret = turret;
    this.desired = desired;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turret.on(CommandConstants.TURRET_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return turret.getPos() < desired+5 && turret.getPos() > desired-5;
  }
}
