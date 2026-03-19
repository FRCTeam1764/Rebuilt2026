// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DefaultCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.CommandConstants;
import frc.robot.subsystems.ShooterWristRev;
import frc.robot.subsystems.StateManager;
import frc.robot.subsystems.TurretRev;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DefaultShooterWristCommand extends Command {
  /** Creates a new DefaultWristCommand. */
  ShooterWristRev turret;
  CommandXboxController xbox;

  public DefaultShooterWristCommand(ShooterWristRev turret, CommandXboxController xbox) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.turret = turret;
    this.xbox = xbox;
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turret.onSpeed(xbox.getRightY());
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
