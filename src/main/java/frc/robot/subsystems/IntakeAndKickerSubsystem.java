// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeAndKickerSubsystem extends SubsystemBase {

  /** Creates a new IntakeAndKickerSubsystem. */
  private static TalonFX intakeMotor;
  final VoltageOut intakeVoltageOutRequest = new VoltageOut(0);

  private static TalonFX kickerMotor;
  final VoltageOut kickerVoltageOutRequest = new VoltageOut(0);
  private DigitalInput intakeBeamBrake;

  public IntakeAndKickerSubsystem() {

    kickerMotor = new TalonFX(24, "rio");
    kickerMotor.getConfigurator();
    // kickerMotor.setNeutralMode(NeutralModeValue.Brake);
    kickerMotor.setInverted(false);

    intakeMotor = new TalonFX(17, "rio");
    intakeMotor.getConfigurator();
    intakeMotor.setInverted(false);
    // intakeMotor.setNeutralMode(NeutralModeValue.Brake);

    intakeMotor.setControl(new Follower(kickerMotor.getDeviceID(), false));

    intakeBeamBrake = new DigitalInput(1);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setIntakeSYSSpeed(double intakeVolts, double kickerVolts) {
    intakeMotor.setVoltage(intakeVolts);
    kickerMotor.setVoltage(kickerVolts);
  }

  public boolean getIntakeSensor() {
    return !intakeBeamBrake.get();
  }

  public boolean getIntakenotBroke() {
    return intakeBeamBrake.get();
  }

}
