// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.VoltageConstants;
import frc.robot.subsystems.IntakeAndKickerSubsystem;
import frc.robot.subsystems.KickerSubsystem;

public class IntakeAndKickerCMD extends Command {
  /** Creates a new IntakeAndKickerCMD. */
  private final IntakeAndKickerSubsystem intakeAndKickerSubsystem;
  private final KickerSubsystem kickerSubsystem;
  private double kickerVolts;
  private double intakeVolts;
  private double timeStart;
  private double currentTime;

  public IntakeAndKickerCMD(IntakeAndKickerSubsystem intakeAndKickerSubsystem, double kickerVolts, double intakeVolts,
      KickerSubsystem kickerSubsystem) {
    this.intakeAndKickerSubsystem = intakeAndKickerSubsystem;
    this.kickerSubsystem = kickerSubsystem;
    this.intakeVolts = intakeVolts;
    this.kickerVolts = kickerVolts;
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(intakeAndKickerSubsystem);
    addRequirements(kickerSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Intake and Kicker SYS started");
    timeStart = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    intakeAndKickerSubsystem.setIntakeSYSSpeed(intakeVolts, kickerVolts);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    currentTime = Timer.getFPGATimestamp();
    if (currentTime - timeStart < 2) {
      kickerSubsystem.setKickerSpeed(VoltageConstants.vk_KickerReverse);
      Timer.delay(.75);
      intakeAndKickerSubsystem.setIntakeSYSSpeed(0, 0);
      kickerSubsystem.setKickerSpeed(0);

    } else {
      intakeAndKickerSubsystem.setIntakeSYSSpeed(0, 0);
      kickerSubsystem.setKickerSpeed(0);
    }

    System.out.println("Intake and Kicker SYS stop");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (intakeAndKickerSubsystem.getIntakeSensor()) {
      return true;
    } else {
      return false;
    }
  }
}