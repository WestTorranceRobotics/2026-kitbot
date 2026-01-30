package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;

public class CommandLogitecJoystickController {
    private GenericHID controller;

    public CommandLogitecJoystickController(int port) {

        controller = new GenericHID(port);
    }

    public double getLeftY() {

        return -controller.getRawAxis(1);

    }

    public double getRightY() {

        return -controller.getRawAxis(5);

    }

    public double getLeftX() {

        return controller.getRawAxis(0);

    }

    public double getRightX() {

        return controller.getRawAxis(4);

    }

}
