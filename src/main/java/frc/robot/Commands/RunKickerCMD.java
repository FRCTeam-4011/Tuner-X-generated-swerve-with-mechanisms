// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.KickerSubsystem;

public class RunKickerCMD extends Command {
  /** Creates a new KickerCommand. */
  private final KickerSubsystem kickerSubsystem;
  private double kickerVolts;

  public RunKickerCMD(KickerSubsystem kickerSubsystem, double kickerVolts) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.kickerSubsystem = kickerSubsystem;

    this.kickerVolts = kickerVolts;

    addRequirements(kickerSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("KickerCMD Started");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    kickerSubsystem.setKickerSpeed(kickerVolts);

  }

  // change

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    kickerSubsystem.setKickerSpeed(0);
    System.out.println("KickerCMD Stopped");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}