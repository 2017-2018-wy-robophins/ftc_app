package org.firstinspires.ftc.teamcode.components.grabber;

import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by efyang on 2/15/18.
 */

public class HingedGrabber implements Grabber {
    private final ServoPair top;
    private final ServoPair hinge;
    private final ServoPair bottom;
    private final int SERVO_WAIT_MS = 300;

    public HingedGrabber(Servo topLeft, Servo topRight, Servo leftHinge, Servo rightHinge, Servo bottomLeft, Servo bottomRight) {
        top = new ServoPair(topLeft, topRight);
        hinge = new ServoPair(leftHinge, rightHinge);
        bottom = new ServoPair(bottomLeft, bottomRight);
        close();
    }

    public void close() {
        top.close();
        bottom.close();
        hinge.open();
    }

    public void open() throws InterruptedException {
        hinge.open();
        top.open();
        Thread.sleep(SERVO_WAIT_MS);
        bottom.open();
    }

    public void top_grab() throws InterruptedException {
        bottom.open();
        top.open();
        hinge.close();
        Thread.sleep(SERVO_WAIT_MS);
        top.close();
        Thread.sleep(SERVO_WAIT_MS);
        hinge.open();
    }

    public void bottom_grab() {
        bottom.open();
    }
}
