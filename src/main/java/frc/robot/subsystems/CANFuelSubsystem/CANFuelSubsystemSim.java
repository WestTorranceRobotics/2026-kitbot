package frc.robot.subsystems.CANFuelSubsystem;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.FuelConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class CANFuelSubsystemSim extends SubsystemBase implements CANFuelSubsystemIO {
    private SparkMax motor = new SparkMax(INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushless);
    private SparkMaxSim launcherMotorSim;
    private SparkMaxConfig launcherConfig = new SparkMaxConfig();

    private SparkMax feederMotor = new SparkMax(FEEDER_MOTOR_ID, MotorType.kBrushless);
    // TODO: implement feeder sim in periodic
    private SparkMaxSim feederMotorSim;

    private FlywheelSim flywheelSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.00038228776, 1), DCMotor.getNEO(1));

    private BangBangController bangbang = new BangBangController();

    @Logged
    private double targetLauncherRPM = 0;

    @Logged
    private double actualMotorRPM = 0;

    @Logged
    private double actualFlywheelRPM = 0;

    public CANFuelSubsystemSim() {
        launcherMotorSim = new SparkMaxSim(motor, DCMotor.getNEO(1));
        launcherConfig.inverted(true);
        launcherConfig.smartCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT);
        motor.configure(launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        feederMotorSim = new SparkMaxSim(feederMotor, DCMotor.getNEO(1));
    }

    @Override
    public void simulationPeriodic() {
        // Set input voltage
        flywheelSim.setInput(launcherMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());

        // Update flywheel
        flywheelSim.update(0.02);

        // Update motor
        launcherMotorSim.iterate(
                launcherMotorSim.getVelocity(), // RPM by default
                RoboRioSim.getVInVoltage(), 0.02);
        actualMotorRPM = launcherMotorSim.getVelocity();
        actualFlywheelRPM = flywheelSim.getAngularVelocityRPM();

        // TODO: might have to move to RobotContainer to sim with every motor?
        RoboRioSim.setVInVoltage(
                BatterySim.calculateDefaultBatteryLoadedVoltage(flywheelSim.getCurrentDrawAmps()));
    }

    public void setLauncherVelocity(AngularVelocity targetVelocity) {
        targetLauncherRPM = targetVelocity.in(RotationsPerSecond) * 60;
        motor.set(bangbang.calculate(
                launcherMotorSim.getVelocity(), targetLauncherRPM));
    }

    public void setLauncherVoltage(Voltage voltage) {
        motor.set(voltage.in(Volts));
    }

    public void setLauncherVoltage(double voltage) {
        motor.set(voltage);
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
}
