package org.firstinspires.ftc.teamcode.components.grabber;

import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by efyang on 2/15/18.
 */

class ServoPair {
    // positioning is based on a frontal view of the robot
    private final Servo right;
    private final Servo left;
    private final double SERVO_OPEN_POS = 1.0;
    private final double SERVO_CLOSED_POS = 0;

    ServoPair(Servo left, Servo right) {
        this.right = right;
        this.left = left;
    }

    private void setPosition(double position) {
        right.setPosition(position);
        left.setPosition(position);
    }

    void close() {
        setPosition(SERVO_CLOSED_POS);
    }

    void open() {
        setPosition(SERVO_OPEN_POS);
    }
}
