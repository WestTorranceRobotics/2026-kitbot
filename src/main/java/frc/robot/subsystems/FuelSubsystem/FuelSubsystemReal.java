// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.FuelSubsystem;

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
public class FuelSubsystemReal extends SubsystemBase implements FuelSubsystemIO {
  private final SparkMax feederMotor = new SparkMax(FEEDER_MOTOR_ID, MotorType.kBrushless);
  private final SparkMax launcherMotor = new SparkMax(INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushless);
  private final BangBangController bangbang = new BangBangController();
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0.00255);

  private double targetRPM = 0;
  private double actualRPM = 0;

  private double feedfowardoutput = 0;


  /** Creates a new CANBallSubsystem. */
  @SuppressWarnings("removal")
  public FuelSubsystemReal() {
    SparkMaxConfig feederConfig = new SparkMaxConfig();
    feederConfig.smartCurrentLimit(FEEDER_MOTOR_CURRENT_LIMIT);
    feederMotor.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig launcherConfig = new SparkMaxConfig();
    launcherConfig.idleMode(IdleMode.kCoast);
    launcherConfig.smartCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT);
    launcherMotor.configure(launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SmartDashboard.putNumber("Intaking feeder roller value", INTAKING_FEEDER_VOLTAGE);
    SmartDashboard.putNumber("Intaking intake roller value", INTAKING_INTAKE_VOLTAGE);
    SmartDashboard.putNumber("Launching feeder roller value", LAUNCHING_FEEDER_VOLTAGE);
    SmartDashboard.putNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_VOLTAGE);
    SmartDashboard.putNumber("Spin-up feeder roller value", SPIN_UP_FEEDER_VOLTAGE);
  }

  // A method to set the speed of the intake roller with bang-bang control
  public void setLauncherVelocity(AngularVelocity velocity) {
    this.targetRPM = velocity.in(RotationsPerSecond) * 60;
    bangbang.setSetpoint(targetRPM);
    launcherMotor
        .setVoltage(
            bangbang.calculate(
                launcherMotor.getEncoder().getVelocity()) * RoboRioDataJNI.getVInVoltage()
                + 0.9 * feedforward.calculate(targetRPM));

    // launcherMotor.setVoltage(feedforward.calculate(targetRPM));
  }

  // A method to set the voltage of the intake roller
  public void setLauncherVoltage(Voltage voltage) {
    targetRPM = 0;
    launcherMotor.set(voltage.in(Volts));
  }

  public void setLauncherVoltage(double voltage) {
    launcherMotor.set(voltage);
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
    launcherMotor.set(0);
  }

  public void stopFeeder() {
    feederMotor.set(0);
  }

  public double feedfowardoutput() {
    return feedfowardoutput;
  }

  @Override
  public void periodic() {
    this.actualRPM = launcherMotor.getEncoder().getVelocity();
  }

  public double getTargetLauncherRPM() {
    return targetRPM;
  }

  public double getActualLauncherRPM() {
    return actualRPM;
  }
}
