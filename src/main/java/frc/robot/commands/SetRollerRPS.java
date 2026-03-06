// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RollerSubsytem;

public class SetRollerRPS extends Command {
  private final double rps;
  
  public SetRollerRPS(RollerSubsytem roller, double rps) {
    this.rps = rps;
    addRequirements(roller);
   
  }


  @Override
  public void initialize() {}


  @Override
  public void execute() {
    Variables.roller.rollerRPS = rps;
  }


  @Override
  public void end(boolean interrupted) {
    Variables.roller.rollerRPS = 0;
  }


  @Override
  public boolean isFinished() {
    return false;
  }
}
