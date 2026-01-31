// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.OperatorConstants.*;

import frc.robot.subsystems.CANFuelSubsystem.CANFuelSubsystem;
import frc.robot.subsystems.CANFuelSubsystem.CANFuelSubsystemIO;
import frc.robot.subsystems.CANFuelSubsystem.CANFuelSubsystemSim;
import frc.robot.subsystems.CANFuelSubsystem.CommandLogitechController;

@Logged
public class RobotContainer {
  private final CANFuelSubsystemIO fuelSubsystem;

  private final CommandLogitechController controller = new CommandLogitechController(
      DRIVER_CONTROLLER_PORT);

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    if (Robot.isReal()) {
      fuelSubsystem = new CANFuelSubsystem();
    } else {
      fuelSubsystem = new CANFuelSubsystemSim();
    }

    configureBindings();
  }

  private void configureBindings() {
    fuelSubsystem.setDefaultCommand(fuelSubsystem.runOnce(() -> {
      // fuelSubsystem.setLauncherVelocity(RotationsPerSecond.of(MathUtil.applyDeadband(controller.getLeftY(), .4)* 3000 / 60)); // 3000 RPM
      fuelSubsystem.setLauncherVoltage(controller.getLeftY() * 0.8);
      fuelSubsystem.setFeederVoltage(MathUtil.applyDeadband(controller.getRightY(), .4) * .75);
    }));

  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
