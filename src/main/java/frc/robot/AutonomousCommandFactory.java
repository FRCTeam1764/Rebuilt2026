// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.waitUntilPosition;
import frc.robot.commands.BasicCommands.IndexCommand;
import frc.robot.commands.BasicCommands.IntakeCommand;
import frc.robot.commands.BasicCommands.IntakeWristCommand;
import frc.robot.commands.BasicCommands.RequestStateChange;
import frc.robot.commands.BasicCommands.ShooterRollersCommand;
import frc.robot.commands.ComplexCommands.returnToIdle;
import frc.robot.commands.DriveCommands.DriveBackward;
import frc.robot.commands.DriveCommands.DriveForward;
import frc.robot.commands.LimelightCommands.LockOnAprilTagAuto;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IndexRollers;
import frc.robot.subsystems.IntakeRollers;
import frc.robot.subsystems.IntakeWrist;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterRollers;
import frc.robot.subsystems.ShooterWrist;
import frc.robot.subsystems.StateManager;
import frc.robot.subsystems.StateManager.States;
import frc.robot.subsystems.Turret;

/** Add your docs here. */
public class AutonomousCommandFactory extends CommandFactory{
    private CommandXboxController pilot;
    private CommandSwerveDrivetrain swerve;
    private StateManager stateManager;
    private ShooterRollers shootRollers;
    private IndexRollers indexRollers;
    private IntakeRollers intakeRollers;
    private LimelightSubsystem limelight1;
    private LimelightSubsystem limelight2;
    private ShooterWrist wrist;
    private Turret turret;
    private IntakeWrist intakeWrist;
    private ClimberSubsystem climber;

    public AutonomousCommandFactory(IntakeWrist intakeWrist, Turret turret, ShooterWrist wrist, LimelightSubsystem limelight2, 
                LimelightSubsystem limelight1, IntakeRollers intakeRollers, IndexRollers indexRollers, 
                ShooterRollers shootRollers, ClimberSubsystem climber, CommandXboxController pilot, CommandSwerveDrivetrain swerve, 
                StateManager stateManager) {
        super(intakeWrist, turret, wrist, limelight2, limelight1, intakeRollers, indexRollers, shootRollers, climber, pilot, swerve, stateManager);
        configAutonomousCommands();
    }
      
    
    public Command HubShootCommand(){
        return new SequentialCommandGroup( new RequestStateChange(States.SHOOT, stateManager),
        new ParallelCommandGroup(
            new AimTurretCommand(),
            new AimWristCommand()
        )
        );
    }
    

    public void configAutonomousCommands() {
        NamedCommands.registerCommand("HubShootCommand", HubShootCommand());
        NamedCommands.registerCommand("HubShootCommand", HubShootCommand());
        NamedCommands.registerCommand("GroundIntakeCommand", GroundIntakeCommand());
        NamedCommands.registerCommand("ClimbCommand", ClimbUpCommand());
        NamedCommands.registerCommand("DriveForward", new DriveForward(swerve));
        NamedCommands.registerCommand("idle", new RequestStateChange(States.IDLE, stateManager));
    }

}
