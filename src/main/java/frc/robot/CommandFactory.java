// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.BasicCommands.ClimberCommand;
import frc.robot.commands.BasicCommands.ClimberCommandSpec;
import frc.robot.commands.BasicCommands.IntakeWristCommand;
import frc.robot.commands.BasicCommands.RollersCommand;
import frc.robot.commands.BasicCommands.ShooterFlywheelCommand;
import frc.robot.commands.DriveCommands.DriveForward;
import frc.robot.commands.LimelightCommands.DriveToTarget;
import frc.robot.commands.LimelightCommands.LockOnAprilTag;
import frc.robot.constants.CommandConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeWristRev;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.RollersSubsystem;
import frc.robot.subsystems.ShooterWristRev;
import frc.robot.subsystems.TurretRev;

//This class will handle all command handling for drivers

/** Add your docs here. */

public class CommandFactory {

    private CommandSwerveDrivetrain swerve;
    private CommandXboxController pilot;
    private RollersSubsystem rollers;
    private LimelightSubsystem turretLimelight;
    private ShooterWristRev wrist;
    private TurretRev turret;
    private IntakeWristRev intakeWrist;

    public CommandFactory(IntakeWristRev intakeWrist, TurretRev turret, ShooterWristRev wrist, LimelightSubsystem turretLimelight, 
                RollersSubsystem rollers,
                CommandXboxController pilot, CommandSwerveDrivetrain swerve) {
        this.pilot = pilot;
        this.swerve = swerve;
        this.rollers = rollers;
        this.turretLimelight = turretLimelight;
        this.wrist = wrist;
        this.turret = turret;
        this.intakeWrist = intakeWrist;
    }

    public Command driveForward() {
        return new ParallelDeadlineGroup(
                    new WaitCommand(0.15), 
                    new DriveForward(swerve));
    }

    public Command ShootRampCommand() {
        return new SequentialCommandGroup( 
            new ParallelDeadlineGroup(
                new ShooterFlywheelCommand(rollers, CommandConstants.SHOOTER_SPEED),
                new WaitCommand(2)),
            new RollersCommand(rollers, true, CommandConstants.INTAKE_IN_SPEED)
        );
    }

    public Command ShootCommand() {
        return new RollersCommand(rollers, true, CommandConstants.INTAKE_IN_SPEED);
    }

    public Command GroundIntakeCommand() {
        return new ParallelCommandGroup(
            new IntakeWristCommand(intakeWrist, CommandConstants.INTAKE_WRIST_DOWN), 
            new RollersCommand(rollers, true, CommandConstants.INTAKE_IN_SPEED));
    }
    
    // public Command ClimbUpCommand(){
    //     return new ClimberCommand(true, climber);
    // }

    // public Command ClimbDownCommand(){
    //     return new ClimberCommand(false, climber);
    // }

    public Command resetPos() {
        return new ParallelCommandGroup(
            new RollersCommand(rollers, 0,0,0, 0),
            new IntakeWristCommand(intakeWrist, CommandConstants.INTAKE_WRIST_IN)
        );
    }

} 
