// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.KnownShoot;
import frc.robot.commands.PassSequence;
import frc.robot.commands.Purge;
import frc.robot.commands.RunIntake;
import frc.robot.commands.SetPivotPosition;
import frc.robot.commands.ShootSequence;
import frc.robot.commands.TurnToTagLive;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FloorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ShooterSubsystem s_shooterSubsystem = new ShooterSubsystem();
  private final FloorSubsystem s_floorSubsystem = new FloorSubsystem();
  private final FeederSubsystem s_feederSubsystem = new FeederSubsystem();
  private final IntakeSubsystem s_intakeSubsystem = new IntakeSubsystem();
  private final PivotSubsystem s_pivotSubsystem = new PivotSubsystem();
  private final LimelightSubsystem s_limelightSubsystem = new LimelightSubsystem();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  public RobotContainer() {
    configureBindings();

    m_robotDrive.setDefaultCommand(
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));
  }

  private void configureBindings() {
    new JoystickButton(m_driverController, XboxController.Button.kStart.value)
      .whileTrue(new InstantCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));

    new Trigger(() -> m_driverController.getRightTriggerAxis() > 0.2)
      .whileTrue(new ShootSequence(s_shooterSubsystem, s_feederSubsystem, s_floorSubsystem, m_robotDrive, s_intakeSubsystem, s_pivotSubsystem));

    new Trigger(() -> m_driverController.getLeftTriggerAxis() > 0.2)
      .whileTrue(new RunIntake(s_intakeSubsystem, s_pivotSubsystem, 30, 115));

    new JoystickButton(m_driverController, XboxController.Button.kY.value)
      .whileTrue(new Purge(s_feederSubsystem, s_shooterSubsystem, s_intakeSubsystem, s_floorSubsystem));

    new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
      .whileTrue(new SetPivotPosition(s_pivotSubsystem, 10));

    new JoystickButton(m_driverController, XboxController.Button.kA.value)
      .whileTrue(new KnownShoot(s_shooterSubsystem, s_feederSubsystem, s_floorSubsystem, m_robotDrive, s_intakeSubsystem, s_pivotSubsystem, 68));

    new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
      .whileTrue(new PassSequence(s_shooterSubsystem, s_feederSubsystem, s_floorSubsystem, m_robotDrive, s_intakeSubsystem, s_pivotSubsystem, 80));

    new JoystickButton(m_driverController, XboxController.Button.kX.value)
      .whileTrue(new TurnToTagLive(m_robotDrive));
  }

  public Command getLeftSideAuto() {
    return null; // add your left side auto commands here
  }

  public Command getMiddleAuto() {
    return null; // add your middle auto commands here
  }

  public Command getRightSideAuto() {
    return null; // add your right side auto commands here
  }

  public DriveSubsystem getDriveSubsystem() {
    return m_robotDrive;
  }
}