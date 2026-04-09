// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.concurrent.CopyOnWriteArrayList;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.subsystems.TurretRev;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.LocalizationSubsystem;
import frc.robot.subsystems.RollersSubsystem;
import frc.robot.subsystems.ShooterWristRev;
import frc.robot.commands.BasicCommands.ClimberCommandSpec;
import frc.robot.commands.BasicCommands.ForceClimberCommand;
import frc.robot.commands.BasicCommands.IntakeWristCommand;
import frc.robot.commands.BasicCommands.RollersCommand;
import frc.robot.commands.BasicCommands.ShooterTurretCommand;
import frc.robot.commands.BasicCommands.ShooterWristCommand;
import frc.robot.commands.BasicCommands.SpindexerCommand;
import frc.robot.commands.DefaultCommands.DefaultShooterWristCommand;
import frc.robot.commands.DefaultCommands.DefaultTurretCommand;
import frc.robot.commands.DriveCommands.DriveRobotCentric;
import frc.robot.commands.LimelightCommands.LockOnAprilTag;
import frc.robot.commands.LimelightCommands.TrackObject;
import frc.robot.constants.CommandConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeWristRev;

public class RobotContainer {
    private double MaxSpeed = 0.7 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = 0.6*RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.06).withRotationalDeadband(MaxAngularRate * 0.05) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric().withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.025).withDriveRequestType(DriveRequestType.OpenLoopVoltage).withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController pilot = new CommandXboxController(0);
    private final CommandXboxController copilot = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();


    
    private final Field2d field = new Field2d();
    
    //subsystems
    private final RollersSubsystem rollers = new RollersSubsystem();
    private final ShooterWristRev shooterWrist = new ShooterWristRev();
    private final TurretRev turret = new TurretRev();
    private final IntakeWristRev intakeWrist = new IntakeWristRev();
    //private final ClimberSubsystem climber = new ClimberSubsystem();
    
    //limelights
    private final LimelightSubsystem localLimelight1 = new LimelightSubsystem("limelight-fourtwo");
    private final LimelightSubsystem localLimelight2 = new LimelightSubsystem("limelight-four");

    
    private final LocalizationSubsystem localization = new LocalizationSubsystem(drivetrain, field, localLimelight1, localLimelight2);
    
    //factories
    private final CommandFactory commandFactory = new CommandFactory(intakeWrist, turret, shooterWrist, localLimelight1, rollers, pilot, drivetrain);
    private final AutonomousCommandFactory autoFactory = new AutonomousCommandFactory(intakeWrist, turret, shooterWrist, localLimelight1, rollers, pilot, drivetrain);
    
    
    private final SendableChooser<Command> chooser;

    public RobotContainer() {
        configureBindings();
        drivetrain.configureAutoBuilder();
        chooser = AutoBuilder.buildAutoChooser("Autonomous");
        SmartDashboard.putData("Autos", chooser);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-Math.pow(pilot.getLeftY(), 2) * (pilot.getLeftY()<0 ? -1: 1)  * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-pilot.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-pilot.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Default Commands
        turret.setDefaultCommand(new DefaultTurretCommand(turret, copilot));
        shooterWrist.setDefaultCommand(new DefaultShooterWristCommand(shooterWrist, copilot));
        
        configureMainBindings();

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private void configureMainBindings() {
        // Drive Controls
        pilot.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        
        //copilot.leftTrigger(.7).whileTrue(drivetrain.applyRequest(()->brake));
        
        pilot.start().whileTrue(commandFactory.resetSpeed());
        copilot.start().whileTrue(commandFactory.resetSpeed());
        
        pilot.rightTrigger().whileTrue(commandFactory.GroundIntakeCommand());
        pilot.rightTrigger().onFalse(commandFactory.resetMidPos());
        
        pilot.pov(0).whileTrue(new IntakeWristCommand(intakeWrist, CommandConstants.INTAKE_WRIST_IN));
        pilot.pov(90).whileTrue(new IntakeWristCommand(intakeWrist, CommandConstants.INTAKE_WRIST_MID));
        pilot.pov(180).whileTrue(new IntakeWristCommand(intakeWrist, CommandConstants.INTAKE_WRIST_DOWN));

        pilot.leftBumper().whileTrue(commandFactory.ShootRampWhileIntakeCommand());
        pilot.leftBumper().onFalse(commandFactory.resetMidPos());

        pilot.x().whileTrue(commandFactory.unJamSpin());
        pilot.x().onFalse(commandFactory.resetSpeed());

        pilot.a().whileTrue(new ShooterTurretCommand(turret, 0));
        pilot.a().onFalse(new InstantCommand(() -> turret.onSpeed(0)));
        
        copilot.rightTrigger().whileTrue(commandFactory.ShootRampCommand());
        copilot.rightTrigger().onFalse(commandFactory.resetMidPos());

        copilot.leftTrigger().whileTrue(commandFactory.ShootRampWithSpitOutCommand());
        copilot.leftTrigger().onFalse(commandFactory.resetMidPos());

        copilot.leftBumper().whileTrue(commandFactory.ShootCommand());
        copilot.leftBumper().onFalse(commandFactory.resetMidPos());

        copilot.back().whileTrue(new RollersCommand(rollers, true, CommandConstants.INTAKE_OUT_SPEED));
        copilot.back().onFalse(commandFactory.resetSpeed());
        
    }

    public Command getAutonomousCommand() {
        return chooser.getSelected();
    }
}
