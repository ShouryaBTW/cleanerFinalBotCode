// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.ObjectInputFilter.Config;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsytem extends SubsystemBase {
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;
  private final VelocityVoltage velocityRequest;

  /** Creates a new ShooterSubsytem. */
  public ShooterSubsytem() {
    leftMotor = new TalonFX(40);
    rightMotor = new TalonFX(41);
    velocityRequest = new VelocityVoltage(0).withSlot(0);
    leftMotor.getConfigurator().apply(Configs.ShooterMotor.ShooterConfig);
    rightMotor.getConfigurator().apply(Configs.ShooterMotor.ShooterConfig);
  }

  public double getLeftShooterSpeed () {
    return leftMotor.getVelocity().getValueAsDouble();
  }
  
  public double getRightShooterSpeed () {
    return rightMotor.getVelocity().getValueAsDouble();
  }

  public boolean atTargetSpeed () {
    return Math.abs(getLeftShooterSpeed() - Variables.Shooter.ShooterRPS()) < 1
     && Math.abs(getRightShooterSpeed() - Variables.Shooter.ShooterRPS()) < 1;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    leftMotor.setControl(velocityRequest.withVelocity(Variables.Shooter.ShooterRPS()));
    rightMotor.setControl(velocityRequest.withVelocity(Variables.Shooter.ShooterRPS()));
    SmartDashboard.putNumber("Variable", Variables.Shooter.ShooterRPS());
    SmartDashboard.putNumber("Left Shooter Speed", getLeftShooterSpeed());
    SmartDashboard.putNumber("Right Shooter Speed", getRightShooterSpeed());
    SmartDashboard.putBoolean("At Target Speed", atTargetSpeed());
  }
}
