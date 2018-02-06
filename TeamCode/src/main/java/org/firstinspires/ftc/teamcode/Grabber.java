package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by efyang on 2/6/18.
 */

class Grabber {
    private class GrabServoPair {
        // positioning is based on a frontal view of the robot
        private final Servo right;
        private final Servo left;
        private final double SERVO_OPEN_POS = 1.0;
        private final double SERVO_CLOSED_POS = 0;

        GrabServoPair(Servo left, Servo right) {
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

    private final GrabServoPair top;
    private final GrabServoPair bottom;
    private final int SERVO_WAIT_MS = 300;
    private boolean open = false;

    Grabber(Servo topLeft, Servo topRight, Servo bottomLeft, Servo bottomRight) {
        top = new GrabServoPair(topLeft, topRight);
        bottom = new GrabServoPair(bottomLeft, bottomRight);
        close();
    }

    void close() {
        top.close();
        bottom.close();
        open = false;
    }

    void open() throws InterruptedException {
        top.open();
        Thread.sleep(SERVO_WAIT_MS);
        bottom.open();
        open = true;
    }
}
