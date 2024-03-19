// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  private static CANSparkFlex armMotor;
  private RelativeEncoder armEncoder;

  public ArmSubsystem() {
    armMotor = new CANSparkFlex(18, MotorType.kBrushless);
    armMotor.setIdleMode(IdleMode.kBrake);
    armMotor.setInverted(false);
    armMotor.setSmartCurrentLimit(80);
    armMotor.setSoftLimit(SoftLimitDirection.kForward, 0.00f);
    armMotor.setSoftLimit(SoftLimitDirection.kReverse, -18.60f);
    armMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    armMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    armMotor.burnFlash();

    armEncoder = armMotor.getEncoder();
    armEncoder.setPosition(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Encoder Position", armEncoder.getPosition());
  }

  public void setArmVoltage(double armVolts) {
    armMotor.setVoltage(armVolts);
  }

  public double getArmEncoderPOS() {
    return armEncoder.getPosition();
  }
}
