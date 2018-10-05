package org.firstinspires.ftc.teamcode.common;

public enum ControlMode {
    Drive, Arm, Dual;

    private ControlMode armToggle, dualToggle;
    static {
        Drive.armToggle = Arm;
        Arm.armToggle = Drive;
        Dual.armToggle = Dual;

        Drive.dualToggle = Dual;
        Arm.dualToggle = Dual;
        Dual.dualToggle = Drive;
    }

    public ControlMode toggleArm() {
        return armToggle;
    }

    public ControlMode toggleDual() {
        return dualToggle;
    }
}
