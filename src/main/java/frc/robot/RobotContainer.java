// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.Commands.IntakeAndKickerCMD;
import frc.robot.Commands.IntakeCMD;
import frc.robot.Commands.MoveArmCMD;
import frc.robot.Commands.RunKickerCMD;
import frc.robot.Commands.RunShooterCMD;
import frc.robot.Commands.shootAmpSCG;
import frc.robot.Commands.shootFromBackPodiumSCG;
import frc.robot.Commands.shootFromPodiumSCG;
import frc.robot.Commands.shootFromSubSCG;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeAndKickerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {
        private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
        private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

        /* Setting up bindings for necessary control of the swerve drive platform */
        private final CommandXboxController driverXboxController = new CommandXboxController(0); // My driver
        private final Joystick operatorJoystick = new Joystick(1);
        private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
        private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
        private final KickerSubsystem kickerSubsystem = new KickerSubsystem();
        private final ArmSubsystem armSubsystem = new ArmSubsystem();
        private final IntakeAndKickerSubsystem intakeAndKickerSubsystem = new IntakeAndKickerSubsystem();
        private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.2).withRotationalDeadband(MaxAngularRate * 0.2) // Add a 10% deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                                                 // driving in open loop
        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
        private final Telemetry logger = new Telemetry(MaxSpeed);

        private void configureBindings() {
                drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
                                drivetrain.applyRequest(() -> drive
                                                .withVelocityX(-driverXboxController.getLeftY() * MaxSpeed) // Drive
                                                                                                            // forward
                                                                                                            // with
                                                // negative Y (forward)
                                                .withVelocityY(-driverXboxController.getLeftX() * MaxSpeed) // Drive
                                                                                                            // left with
                                                                                                            // negative
                                                                                                            // X (left)
                                                .withRotationalRate(-driverXboxController.getRightX() * MaxAngularRate) // Drive
                                                                                                                        // counterclockwise
                                                                                                                        // with
                                                                                                                        // negative
                                                                                                                        // X
                                                                                                                        // (left)
                                ));

                driverXboxController.a().whileTrue(drivetrain.applyRequest(() -> brake));
                driverXboxController.b().whileTrue(drivetrain
                                .applyRequest(() -> point.withModuleDirection(
                                                new Rotation2d(-driverXboxController.getLeftY(),
                                                                -driverXboxController.getLeftX()))));

                // reset the field-centric heading on left bumper press
                driverXboxController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

                if (Utils.isSimulation()) {
                        drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
                }
                drivetrain.registerTelemetry(logger::telemeterize);

                driverXboxController.rightBumper()
                                .whileTrue(new IntakeCMD(intakeSubsystem, VoltageConstants.vk_IntakeReverse));
                // driver.y().onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));

                // zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));

                new JoystickButton(operatorJoystick, 1).onTrue(new IntakeAndKickerCMD(intakeAndKickerSubsystem,
                                VoltageConstants.vk_IntakeForward, VoltageConstants.vk_KickerForward, kickerSubsystem));

                new JoystickButton(operatorJoystick, 2)
                                .onTrue(new shootFromPodiumSCG(armSubsystem, shooterSubsystem, kickerSubsystem));

                new JoystickButton(operatorJoystick, 8)
                                .onTrue(new shootFromSubSCG(shooterSubsystem, armSubsystem, kickerSubsystem));

                new JoystickButton(operatorJoystick, 4)
                                .onTrue(new shootAmpSCG(armSubsystem, shooterSubsystem, kickerSubsystem));

                new JoystickButton(operatorJoystick, 7)
                                .onTrue(new shootFromBackPodiumSCG(armSubsystem, shooterSubsystem, kickerSubsystem));

                new JoystickButton(operatorJoystick, 11)
                                .whileTrue(new MoveArmCMD(armSubsystem, VoltageConstants.vk_ArmUp));
                new JoystickButton(operatorJoystick, 12)
                                .whileTrue(new MoveArmCMD(armSubsystem, VoltageConstants.vk_ArmDown));

                new JoystickButton(operatorJoystick, 6)
                                .whileTrue(new IntakeCMD(intakeSubsystem, VoltageConstants.vk_IntakeForward));

                new JoystickButton(operatorJoystick, 5)
                                .whileTrue(new RunKickerCMD(kickerSubsystem, VoltageConstants.vk_KickerForward));

                new JoystickButton(operatorJoystick, 10)
                                .whileTrue(new RunKickerCMD(kickerSubsystem, VoltageConstants.vk_KickerReverse));

                new JoystickButton(operatorJoystick, 9).whileTrue(new RunShooterCMD(shooterSubsystem,
                                VoltageConstants.vk_TopShooterForward, VoltageConstants.vk_BottomShooterForward));

        }

        public RobotContainer() {
                configureBindings();

        }

        public Command getAutonomousCommand() {
                return Commands.print("No autonomous command configured");
        }
}
