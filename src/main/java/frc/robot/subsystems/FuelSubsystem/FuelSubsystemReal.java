// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.FuelSubsystem;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
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
import frc.robot.util.CustomUnits;

import static frc.robot.Constants.FuelConstants.*;
import static edu.wpi.first.units.Units.*;

@Logged
public class FuelSubsystemReal extends SubsystemBase implements FuelSubsystemIO {
  private final SparkMax feederMotor = new SparkMax(FEEDER_MOTOR_ID, MotorType.kBrushless);
  private final SparkMax launcherMotorLeader = new SparkMax(INTAKE_MOTOR_1_ID, MotorType.kBrushless);
  private final SparkMax launcherMotorFollower = new SparkMax(INTAKE_MOTOR_2_ID, MotorType.kBrushless);

  private final BangBangController bangbang = new BangBangController();
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0.00255);

  private double targetRPM = 0;
  private double actualRPM = 0;

  private double feedfowardoutput = 0;

  /** Creates a new CANBallSubsystem. */
  public FuelSubsystemReal() {
    SparkMaxConfig feederConfig = new SparkMaxConfig();
    feederConfig.smartCurrentLimit(FEEDER_MOTOR_CURRENT_LIMIT);
    feederMotor.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // FIXME: The motor inversions are possibly incorrect and need to be checked.
    // DON'T RUN THIS CODE WITH BOTH MOTORS PLUGGED IN before you do that
    SparkMaxConfig launcherLeaderConfig = new SparkMaxConfig();
    launcherLeaderConfig.idleMode(IdleMode.kCoast);
    launcherLeaderConfig.smartCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT);
    launcherLeaderConfig.inverted(false);
    launcherMotorLeader.configure(launcherLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig launcherFollowerConfig = new SparkMaxConfig();
    launcherFollowerConfig.idleMode(IdleMode.kCoast);
    // launcherFollowerConfig.smartCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT);
    // launcherFollowerConfig.inverted(false);
    // launcherFollowerConfig.follow(launcherMotorLeader);
    launcherMotorFollower.configure(launcherFollowerConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    SmartDashboard.putNumber("Intaking feeder roller value", INTAKING_FEEDER_VOLTAGE);
    SmartDashboard.putNumber("Intaking intake roller value", INTAKING_INTAKE_VOLTAGE);
    SmartDashboard.putNumber("Launching feeder roller value", LAUNCHING_FEEDER_VOLTAGE);
    SmartDashboard.putNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_VOLTAGE);
    SmartDashboard.putNumber("Spin-up feeder roller value", SPIN_UP_FEEDER_VOLTAGE);
  }

  // A method to set the speed of the intake roller with bang-bang control
  public void setLauncherVelocity(AngularVelocity velocity) {
    this.targetRPM = velocity.in(CustomUnits.RotationsPerMinute);
    bangbang.setSetpoint(targetRPM);
    launcherMotorLeader
        .setVoltage(
            bangbang.calculate(
                launcherMotorLeader.getEncoder().getVelocity()) * RoboRioDataJNI.getVInVoltage()
                + 0.9 * feedforward.calculate(targetRPM));

    // launcherMotor.setVoltage(feedforward.calculate(targetRPM));
  }

  // A method to set the voltage of the intake roller
  public void setLauncherVoltage(Voltage voltage) {
    targetRPM = 0;
    launcherMotorLeader.set(voltage.in(Volts));
  }

  public void setLauncherVoltage(double voltage) {
    launcherMotorLeader.set(voltage);
  }

  // A method to set the voltage of the intake roller
  public void setFeederVoltage(Voltage voltage) {
    feederMotor.set(voltage.in(Volts));
  }

  public void setFeederVoltage(double voltage) {
    feederMotor.set(voltage);
  }

  // A method to stop the rollers
  public void stopLauncher() {
    launcherMotorLeader.set(0);
  }

  public void stopFeeder() {
    feederMotor.set(0);
  }

  public double feedfowardoutput() {
    return feedfowardoutput;
  }

  @Override
  public void periodic() {
    this.actualRPM = launcherMotorLeader.getEncoder().getVelocity();
  }

  public double getTargetLauncherRPM() {
    return targetRPM;
  }

  public double getActualLauncherRPM() {
    return actualRPM;
  }
}
