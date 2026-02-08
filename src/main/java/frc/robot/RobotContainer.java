// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.OperatorConstants.*;

import frc.robot.subsystems.CommandLogitechController;
import frc.robot.subsystems.FuelSubsystem.FuelSubsystemReal;
import frc.robot.subsystems.FuelSubsystem.FuelSubsystemIO;
import frc.robot.subsystems.FuelSubsystem.FuelSubsystemSim;
import static frc.robot.util.CustomUnits.*;

@Logged
public class RobotContainer {
  private final FuelSubsystemIO fuelSubsystem;

  private final CommandLogitechController controller = new CommandLogitechController(
      DRIVER_CONTROLLER_PORT);

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    if (Robot.isReal()) {
      fuelSubsystem = new FuelSubsystemReal();
    } else {
      fuelSubsystem = new FuelSubsystemSim();
    }

    configureBindings();
  }

  private void configureBindings() {

    controller.y().onTrue(fuelSubsystem.runOnce(() -> {
      fuelSubsystem.setLauncherVelocity(RotationsPerMinute.of(3500));
    })).onFalse(fuelSubsystem.runOnce(() -> {
      fuelSubsystem.stopLauncher();
    }));

    controller.a().onTrue(fuelSubsystem.runOnce(() -> {
      fuelSubsystem.setLauncherVelocity(RotationsPerMinute.of(3000));
    })).onFalse(fuelSubsystem.runOnce(() -> {
      fuelSubsystem.stopLauncher();
    }));

    controller.b().onTrue(fuelSubsystem.runOnce(() -> {
      fuelSubsystem.setLauncherVelocity(RotationsPerMinute.of(2000));
    })).onFalse(fuelSubsystem.runOnce(() -> {
      fuelSubsystem.stopLauncher();
    }));

    // fuelSubsystem.setDefaultCommand(fuelSubsystem.runOnce(() -> {
    // fuelSubsystem
    // .setLauncherVelocity(
    // RotationsPerSecond.of(
    // MathUtil.applyDeadband(
    // controller.getLeftY(), 0.2) * 3000 / 60)); // 3000 RPM
    // fuelSubsystem.setFeederVoltage(MathUtil.applyDeadband(controller.getRightY(),
    // .4) * .75);
    // }));

  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
