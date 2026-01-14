// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// AGAIN REQUEST ELLIOTT IF ANYTHING IS WRONG

package frc.robot.commands.BasicCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterRollers;
//import frc.robot.subsystems.IntakeWrist;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShooterRollersCommand extends Command {
  /** Creates a new ShooterRollersCommand. */
  ShooterRollers shooter;
  double speed;
  boolean stopAtLimit;
  public ShooterRollersCommand(ShooterRollers shooter, double speed, boolean stopAtLimit) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.speed = speed;
    this.stopAtLimit = stopAtLimit;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.wheelsShooter(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.wheelsShooter(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}