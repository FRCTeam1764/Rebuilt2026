// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ComplexCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.BasicCommands.ShooterTurretCommand;
import frc.robot.commands.BasicCommands.ShooterWristCommand;
import frc.robot.subsystems.LocalizationSubsystem;
import frc.robot.subsystems.TurretRev;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AimAtHubTurret extends SequentialCommandGroup {
  /** Creates a new AimAtHub. */
  public AimAtHubTurret(TurretRev turretRev, LocalizationSubsystem localization) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    double turretPos = localization.getRobotRelHubAngle();
    addCommands(
      new ParallelCommandGroup(
        new ShooterTurretCommand(turretRev, turretPos)
      )
    );
  }
}
