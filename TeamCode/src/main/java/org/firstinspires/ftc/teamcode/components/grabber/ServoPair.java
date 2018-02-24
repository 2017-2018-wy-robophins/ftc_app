package org.firstinspires.ftc.teamcode.components.grabber;

import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by efyang on 2/15/18.
 */

class ServoPair {
    // positioning is based on a frontal view of the robot
    private final Servo right;
    private final Servo left;
    private final double SERVO_OPEN_POS;
    private final double SERVO_CLOSED_POS;

    ServoPair(Servo left, Servo right, double min, double max) {
        this.right = right;
        this.left = left;
        this.SERVO_OPEN_POS = max;
        this.SERVO_CLOSED_POS = min;
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
