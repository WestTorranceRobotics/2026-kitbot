package frc.robot.subsystems;

import edu.wpi.first.wpilibj.GenericHID;

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
}
