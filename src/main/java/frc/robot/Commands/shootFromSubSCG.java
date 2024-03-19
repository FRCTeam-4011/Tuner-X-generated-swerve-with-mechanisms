// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

//import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.VoltageConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class shootFromSubSCG extends SequentialCommandGroup {
  // Creates a new shootFromSubSCG.
  public shootFromSubSCG(ShooterSubsystem shooterSubsystem, ArmSubsystem armSubsystem, KickerSubsystem kickerSubsystem)

  {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

        new MoveArmToSubShotCMD(armSubsystem, VoltageConstants.vk_ArmUp).withTimeout(2.5), //
        new RunShooterTimeBaseCMD(shooterSubsystem, VoltageConstants.vk_TopShooterSub,
            VoltageConstants.vk_BottomShooterSub)
            .alongWith(new RunKickerTimeBaseCMD(kickerSubsystem, VoltageConstants.vk_KickerForward)).withTimeout(2.5), //

        // new RunShooterCMD(shooterSubsystem, 0, 0).withTimeout(.5),//
        // new RunKickerCMD(kickerSubsystem, 0).withTimeout(.5),//
        new MoveArmToHomePOSCMD(armSubsystem, VoltageConstants.vk_ArmDown)//

    );

  }
}
