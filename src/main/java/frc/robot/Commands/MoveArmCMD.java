// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class MoveArmCMD extends Command {
  /** Creates a new MoveArmCMD. */
  private final ArmSubsystem armSubsystem;
  private double armVolts;

  public MoveArmCMD(ArmSubsystem armSubsystem, double armVolts) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.armSubsystem = armSubsystem;
    this.armVolts = armVolts;

    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Move Arm Manually Start");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armSubsystem.setArmVoltage(armVolts);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.setArmVoltage(0);
    System.out.println("Move Arm Manually Stop");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}