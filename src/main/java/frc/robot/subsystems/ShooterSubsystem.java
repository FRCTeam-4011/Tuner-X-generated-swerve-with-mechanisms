// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  private static CANSparkMax topShooterMotor;
  private static RelativeEncoder topEncoder;
  private static CANSparkMax bottomShooterMotor;

  public ShooterSubsystem() {
    topShooterMotor = new CANSparkMax(21, MotorType.kBrushless);

    topShooterMotor.setInverted(true);
    topShooterMotor.setSmartCurrentLimit(80);
    topShooterMotor.burnFlash();

    topEncoder = topShooterMotor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);

    bottomShooterMotor = new CANSparkMax(22, MotorType.kBrushless);

    bottomShooterMotor.setSmartCurrentLimit(80);
    bottomShooterMotor.setInverted(false);
    bottomShooterMotor.burnFlash();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Top Shooter Velocity", topEncoder.getVelocity());
  }

  public void setShooterSpeed(double topShooterVolts, double bottomShooterVolts) {
    topShooterMotor.setVoltage(topShooterVolts);
    bottomShooterMotor.setVoltage(bottomShooterVolts);
  }

  public double getShooterVelocity() {
    return (topEncoder.getVelocity());
  }
  // public double getShooterVelocity(){
  // return(bottomEncoder.getVelocity());
  // }

}