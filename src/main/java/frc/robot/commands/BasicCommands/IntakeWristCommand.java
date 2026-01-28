// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BasicCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.constants.CommandConstants;
import frc.robot.subsystems.IntakeWrist;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeWristCommand extends Command {
  /** Creates a new IntakeWristCommand. */
  int desired;
  IntakeWrist intakeWrist;

  public IntakeWristCommand(IntakeWrist intakeWrist2, double intakeWristSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.desired = intakeWrist2;
    this.intakeWrist = intakeWristSpeed;
    addRequirements(intakeWristSpeed);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeWrist.flex(CommandConstants.INTAKE_WRIST_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeWrist.stopFlex();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.intakeWrist.getEncoderPos() <= this.desired+1 && this.intakeWrist.getEncoderPos() >= this.desired-1;

  } 
}
                                                                                                           