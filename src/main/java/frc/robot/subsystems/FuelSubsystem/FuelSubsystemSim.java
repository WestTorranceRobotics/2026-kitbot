package frc.robot.subsystems.FuelSubsystem;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.FuelConstants.*;
import static frc.robot.util.CustomUnits.RotationsPerMinute;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class FuelSubsystemSim extends SubsystemBase implements FuelSubsystemIO {
    private SparkMax launcherMotor = new SparkMax(INTAKE_MOTOR_1_ID, MotorType.kBrushless);
    private SparkMaxSim launcherMotorSim;
    private SparkMaxConfig launcherConfig = new SparkMaxConfig();
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0.00255);

    private SparkMax feederMotor = new SparkMax(FEEDER_MOTOR_ID, MotorType.kBrushless);
    // TODO: implement feeder sim in periodic
    private SparkMaxSim feederMotorSim;

    private FlywheelSim flywheelSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.00062156662, 1), DCMotor.getNEO(1));

    private BangBangController bangbang = new BangBangController();

    private double targetRPM = 0;
    private double actualRPM = 0;
    
    @Logged
    private double actualMotorRPM = 0; 

    public FuelSubsystemSim() {
        launcherMotorSim = new SparkMaxSim(launcherMotor, DCMotor.getNEO(1));
        launcherConfig.inverted(true);
        launcherConfig.smartCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT);
        launcherMotor.configure(launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        feederMotorSim = new SparkMaxSim(feederMotor, DCMotor.getNEO(1));
    }

    @Override
    public void periodic() {
        // Set input voltage
        flywheelSim.setInput(launcherMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());

        // Update flywheel
        flywheelSim.update(0.02);

        // Update motor
        launcherMotorSim.iterate(
                launcherMotor.getEncoder().getVelocity(), // RPM by default
                RoboRioSim.getVInVoltage(), 0.02);
        this.actualMotorRPM = launcherMotorSim.getVelocity();
        this.actualRPM = flywheelSim.getAngularVelocityRPM();

        // TODO: might have to move to RobotContainer to sim with every motor?
        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(flywheelSim.getCurrentDrawAmps()));
    }

  public void setLauncherVelocity(AngularVelocity velocity) {
    this.targetRPM = velocity.in(RotationsPerMinute);
    bangbang.setSetpoint(targetRPM);
    launcherMotor
        .setVoltage(
            bangbang.calculate(
                launcherMotor.getEncoder().getVelocity()) * RoboRioDataJNI.getVInVoltage()
                + 0.9 * feedforward.calculate(targetRPM));
  }

    public void setLauncherVelocity(double RPM) {
        this.targetRPM = RPM;

        bangbang.setSetpoint(targetRPM);
        launcherMotor.setVoltage(bangbang.calculate(
                launcherMotor.getEncoder().getVelocity()) * RoboRioSim.getVInVoltage());
    }

    public void setLauncherVoltage(Voltage voltage) {
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

    public void stopLauncher() {
        setLauncherVelocity(RotationsPerSecond.zero());
    }

    public void stopFeeder() {
        feederMotor.set(0);
    }

    public double getTargetLauncherRPM() {
        return targetRPM;
    }

    public double getActualLauncherRPM() {
        return actualRPM;
    }
}
