// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.KickerSubsystem;

public class RunKickerTimeBaseCMD extends Command {
  /** Creates a new RunKickerTimeBaseCMD. */
  private final KickerSubsystem kickerSubsystem;
  private double kickerVolts;
  private double startTime;
  private double currentTime;

  public RunKickerTimeBaseCMD(KickerSubsystem kickerSubsystem, double kickerVolts) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.kickerSubsystem = kickerSubsystem;

    this.kickerVolts = kickerVolts;

    addRequirements(kickerSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Run Kicker Time Base Start");
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentTime = Timer.getFPGATimestamp();
    if (currentTime - startTime > 1) {

      kickerSubsystem.setKickerSpeed(kickerVolts);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    kickerSubsystem.setKickerSpeed(0);
    System.out.println("Run Kicker Time Base Stop");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}