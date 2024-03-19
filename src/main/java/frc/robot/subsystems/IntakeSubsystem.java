// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private static TalonFX intakeMotor;
  final VoltageOut intakeVoltageOutRequest = new VoltageOut(0);

  public IntakeSubsystem() {
    intakeMotor = new TalonFX(17, "rio");
    intakeMotor.getConfigurator();
    intakeMotor.setInverted(false);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setIntakeVoltage(double intakeAloneVolts) {
    intakeMotor.setVoltage(intakeAloneVolts);
  }
}
