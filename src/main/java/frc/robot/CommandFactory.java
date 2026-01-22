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
import frc.robot.commands.waitUntilPosition;
import frc.robot.commands.waitUntilPositionIndex;
import frc.robot.commands.BasicCommands.IndexCommand;
import frc.robot.commands.BasicCommands.IntakeCommand;
import frc.robot.commands.BasicCommands.RequestStateChange;
import frc.robot.commands.BasicCommands.ShooterRollersCommand;
import frc.robot.commands.ComplexCommands.returnToIdle;
import frc.robot.commands.DriveCommands.DriveForward;
import frc.robot.commands.LimelightCommands.DriveToTarget;
import frc.robot.commands.LimelightCommands.DriveToTargetOffset;
import frc.robot.commands.LimelightCommands.DriveToTargetOffsetLL3;
import frc.robot.commands.LimelightCommands.LockOnAprilTag;
import frc.robot.commands.LimelightCommands.TurnToAngle;
import frc.robot.constants.CommandConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IndexRollers;
import frc.robot.subsystems.IntakeRollers;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterRollers;
import frc.robot.subsystems.ShooterWrist;
import frc.robot.subsystems.StateManager;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.StateManager.States;

//This class will handle all command handling for drivers

/** Add your docs here. */

public class CommandFactory {

    private CommandSwerveDrivetrain swerve;
    private StateManager stateManager;
    private CommandXboxController pilot;
    private ShooterRollers shootRollers;
    private IndexRollers indexRollers;
    private IntakeRollers intakeRollers;
    private LimelightSubsystem localLimelight;
    private LimelightSubsystem turretLimelight;
    private ShooterWrist wrist;
    private Turret turret;

    private double intakeSpeed = CommandConstants.GROUND_INTAKE_ROLLERS_SPEED;

    public CommandFactory(Turret turret, ShooterWrist wrist, LimelightSubsystem turretLimelight, 
                LimelightSubsystem localLimelight, IntakeRollers intakeRollers, IndexRollers indexRollers, 
                ShooterRollers shootRollers, CommandXboxController pilot, CommandSwerveDrivetrain swerve, 
                StateManager stateManager) {
        this.pilot = pilot;
        this.swerve = swerve;
        this.stateManager = stateManager;
        this.shootRollers = shootRollers;
        this.indexRollers = indexRollers;
        this.intakeRollers = intakeRollers;
        this.localLimelight = localLimelight;
        this.turretLimelight = turretLimelight;
        this.wrist = wrist;
        this.turret = turret;
    }

    public Command driveForward() {
        return new ParallelDeadlineGroup(
                    new WaitCommand(0.15), 
                    new DriveForward(swerve));
    }

    public Command HubShootCommand(){
        return new ParallelCommandGroup(
            new AimTurretCommand(),
            new AimWristCommand(), 
            new ShooterRollersCommand(shootRollers, 0.2)
            // add hubshoot values
        );
    }

    public Command GroundIntakeCommand(){ 
        return new ParallelCommandGroup(
            new IntakeCommand(intakeRollers, intakeSpeed),
            new IndexCommand(indexRollers, intakeSpeed)
        );
    }
    // add ground intake values

    public Command interupted(boolean wasInteruppted) {
        if (wasInteruppted) {
            return new InstantCommand();
        }
        return new returnToIdle(stateManager);
    }

    //desiredAction might not be used
    public enum desiredAction {
        IDLE
    }
    public desiredAction currentAction = desiredAction.IDLE;

} 
