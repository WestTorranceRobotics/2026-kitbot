package frc.robot.subsystems.FuelSubsystem;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Subsystem;

@Logged
public interface FuelSubsystemIO extends Subsystem {
    public void setLauncherVelocity(AngularVelocity velocity);

    public void setLauncherVoltage(Voltage voltage);

    public void setLauncherVoltage(double voltage);

    public void setFeederVoltage(Voltage voltage);

    public void setFeederVoltage(double voltage);

    public void stopLauncher();

    public void stopFeeder();

    public double getTargetLauncherRPM();

    public double getActualLauncherRPM();
}
