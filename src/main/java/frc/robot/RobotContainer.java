// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.RPM;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.DriveIOSparkMax;
import frc.robot.subsystems.Intake.Roller;
import frc.robot.subsystems.Intake.RollerIOSparkMax;

public class RobotContainer {
  private Drive drivebase_;
  private Roller roller_;
  
  private final CommandXboxController gamepad_ = new CommandXboxController(0);

  private final LoggedDashboardChooser<Command> autoChooser_;

  public RobotContainer() {
    roller_ = new Roller(new RollerIOSparkMax());
    drivebase_ = new Drive(new DriveIOSparkMax());

    drivebase_.setDefaultCommand(drivebase_.joystickDrive(
      drivebase_,
      () -> gamepad_.getLeftY(),
      () -> gamepad_.getLeftX(),
      () -> gamepad_.getRightX()
    ));

    autoChooser_ = new LoggedDashboardChooser<>("Auto Choices");

    autoChooser_.addDefaultOption("Collect and Score", AutoCommands.score(drivebase_, roller_));
    autoChooser_.addOption("Collect", AutoCommands.collect(drivebase_, roller_));

    configureBindings();
  }

  private void configureBindings() {

    gamepad_.leftTrigger().whileTrue(roller_.runRollers(RPM.of(2000000000)));
    gamepad_.rightTrigger().whileTrue(roller_.runRollers(RPM.of(-2000000000)));
  }

  public Command getAutonomousCommand() {
    return autoChooser_.get();
  }
}
