// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.ObjectInputFilter.Config;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RollerSubsytem extends SubsystemBase {
  private final TalonFX rollerMotor;
  private final VelocityVoltage velocityRequest;

  /** Creates a new RollerSubsytem. */
  public RollerSubsytem() {
    rollerMotor = new TalonFX(30);
    velocityRequest = new VelocityVoltage(0).withSlot(0);
    rollerMotor.getConfigurator().apply(Configs.rollerMotor.rollerConfig);
  }

  public double getRollerSpeed () {
    return rollerMotor.getVelocity().getValueAsDouble();
  }

  public boolean atTargetSpeed () {
    return Math.abs(getRollerSpeed() - Variables.roller.rollerRPS()) < 1;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    rollerMotor.setControl(velocityRequest.withVelocity(Variables.roller.rollerRPS()));
    SmartDashboard.putNumber("Variable", Variables.roller.rollerRPS());
    SmartDashboard.putBoolean("At Target Speed", atTargetSpeed());
  }
}
