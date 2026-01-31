// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.CANFuelSubsystem;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.FuelConstants.*;
import static edu.wpi.first.units.Units.*;

@Logged
public class CANFuelSubsystem extends SubsystemBase implements CANFuelSubsystemIO {
  private final SparkMax feederRoller;
  private final SparkMax intakeLauncherRoller;
  private final BangBangController bangbang = new BangBangController();
  private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0, 0);

  @Logged
  double actualRPM = 0;

  @Logged
  double targetRPM = 0;

  /** Creates a new CANBallSubsystem. */
  public CANFuelSubsystem() {
    intakeLauncherRoller = new SparkMax(INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushless);
    feederRoller = new SparkMax(FEEDER_MOTOR_ID, MotorType.kBrushless);

    SparkMaxConfig feederConfig = new SparkMaxConfig();
    feederConfig.smartCurrentLimit(FEEDER_MOTOR_CURRENT_LIMIT);
    feederRoller.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig launcherConfig = new SparkMaxConfig();
    launcherConfig.idleMode(IdleMode.kCoast);
    launcherConfig.smartCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT);
    intakeLauncherRoller.configure(launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    bangbang.setTolerance(100);

    SmartDashboard.putNumber("Intaking feeder roller value", INTAKING_FEEDER_VOLTAGE);
    SmartDashboard.putNumber("Intaking intake roller value", INTAKING_INTAKE_VOLTAGE);
    SmartDashboard.putNumber("Launching feeder roller value", LAUNCHING_FEEDER_VOLTAGE);
    SmartDashboard.putNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_VOLTAGE);
    SmartDashboard.putNumber("Spin-up feeder roller value", SPIN_UP_FEEDER_VOLTAGE);
  }

  // A method to set the speed of the intake roller with bang-bang control
  public void setLauncherVelocity(AngularVelocity velocity) {
    targetRPM = velocity.in(RotationsPerSecond) * 60;
    intakeLauncherRoller
        .setVoltage(bangbang.calculate(intakeLauncherRoller.getEncoder().getVelocity(),
            targetRPM) * RoboRioDataJNI.getVInVoltage() + 0.9 * ff.calculate(targetRPM));
  }

  // A method to set the voltage of the intake roller
  public void setLauncherVoltage(Voltage voltage) {
    intakeLauncherRoller.set(voltage.in(Volts));
  }

  public void setLauncherVoltage(double voltage) {
    intakeLauncherRoller.set(voltage);
  }

  // A method to set the voltage of the intake roller
  public void setFeederVoltage(Voltage voltage) {
    feederRoller.set(voltage.in(Volts));
  }

  public void setFeederVoltage(double voltage) {
    feederRoller.set(voltage);
  }

  // A method to stop the rollers
  public void stopLauncher() {
    intakeLauncherRoller.set(0);
  }

  public void stopFeeder() {
    feederRoller.set(0);
  }

  @Override
  public void periodic() {
    actualRPM = intakeLauncherRoller.getEncoder().getVelocity();
    // This method will be called once per scheduler run
  }
}
