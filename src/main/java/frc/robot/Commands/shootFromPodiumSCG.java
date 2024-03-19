// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.VoltageConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class shootFromPodiumSCG extends SequentialCommandGroup {
  // Creates a new shootFromPodiumSCG.
  public shootFromPodiumSCG(ArmSubsystem armSubsystem, ShooterSubsystem shooterSubsystem,
      KickerSubsystem kickerSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        /*
         * new ParallelCommandGroup(
         * new RunShooterCMD(shooterSubsystem, VoltageConstants.vk_TopShooterForward,
         * VoltageConstants.vk_BottomShooterForward),//
         * new MoveArmToPodShotCMD(armSubsystem, VoltageConstants.vk_ArmUp)
         * ),//
         */
        new MoveArmToPodShotCMD(armSubsystem, VoltageConstants.vk_ArmUp).withTimeout(2), //
        new RunShooterTimeBaseCMD(shooterSubsystem, VoltageConstants.vk_TopShooterForward,
            VoltageConstants.vk_BottomShooterForward)
            .alongWith(new RunKickerTimeBaseCMD(kickerSubsystem, VoltageConstants.vk_KickerForward)).withTimeout(3), //

        new MoveArmToHomePOSCMD(armSubsystem, VoltageConstants.vk_ArmDown)//
    );

  }
}
