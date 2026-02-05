package frc.robot.subsystems;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CommandLogitechController {
    private GenericHID controller;

    public CommandLogitechController(int port) {
        controller = new GenericHID(port);
    }

    public double getLeftX() {
        return controller.getRawAxis(0);
    }

    public double getLeftY() {
        return -controller.getRawAxis(1);
    }

    public double getRightX() {
        return controller.getRawAxis(4);
    }

    public double getRightY() {
        return -controller.getRawAxis(5);
    }

    public Trigger getA() {
        return new Trigger(() -> controller.getRawButton(1));
    }

    public Trigger getB() {
        return new Trigger(() -> controller.getRawButton(2));
    }

    public Trigger getY() {
        return new Trigger(() -> controller.getRawButton(4));
    }

    public Trigger getX() {
        return new Trigger(() -> controller.getRawButton(3));
    }
}
