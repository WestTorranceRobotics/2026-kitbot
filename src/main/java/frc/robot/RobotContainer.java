// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.OperatorConstants.*;

import frc.robot.subsystems.CommandLogitechController;
import frc.robot.subsystems.FuelSubsystem.FuelSubsystemReal;
import frc.robot.subsystems.FuelSubsystem.FuelSubsystemIO;
import frc.robot.subsystems.FuelSubsystem.FuelSubsystemSim;

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

    controller.getA().onTrue(fuelSubsystem.runOnce(() -> {
      fuelSubsystem.setLauncherVelocity(RotationsPerSecond.of(3000 / 60));
    })).onFalse(fuelSubsystem.runOnce(() -> {
      fuelSubsystem.stopLauncher();
    }));
  }

  private void configureBindings() {
    // fuelSubsystem.setDefaultCommand(fuelSubsystem.runOnce(() -> {
    //   fuelSubsystem
    //       .setLauncherVelocity(
    //           RotationsPerSecond.of(
    //               MathUtil.applyDeadband(
    //                   controller.getLeftY(), 0.2) * 3000 / 60)); // 3000 RPM
    //   // fuelSubsystem.setLauncherVoltage(controller.getLeftY() * 0.8);
    //   fuelSubsystem.setFeederVoltage(MathUtil.applyDeadband(controller.getRightY(), .4) * .75);
    // }));

  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
