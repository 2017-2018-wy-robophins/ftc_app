package org.firstinspires.ftc.teamcode.common;

public class ControlState {
    private ControlMode controlMode;

    public ControlState() {
        this(ControlMode.Drive);
    }

    public ControlState(ControlMode controlMode) {
        this.controlMode = controlMode;
    }

    public void toggleArm() {
        controlMode = controlMode.toggleArm();
    }

    public void toggleDual() {
        controlMode = controlMode.toggleDual();
    }

    public ControlMode getControlMode() {
        return controlMode;
    }
}
