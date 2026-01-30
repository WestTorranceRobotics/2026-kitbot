package frc.robot.subsystems;

import edu.wpi.first.wpilibj.GenericHID;

public class CommandLogitecJoystickController {
    private GenericHID controller;

    CommandLogitecJoystickController(int port) {

        controller = new GenericHID(port);
    }

    public double getLeftY() {

        return controller.getRawAxis(1);

    }

    public double getRightY() {

        return controller.getRawAxis(3);

    }

    public double getLeftX() {

        return controller.getRawAxis(1);

    }

    public double getRightX() {

        return controller.getRawAxis(2);

    }

}
