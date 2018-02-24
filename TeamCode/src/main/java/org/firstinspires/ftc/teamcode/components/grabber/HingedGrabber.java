package org.firstinspires.ftc.teamcode.components.grabber;

import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by efyang on 2/15/18.
 */

public class HingedGrabber implements Grabber {
    private final ServoPair top;
    private final ServoPair hinge;
    private final ServoPair bottom;
    private final int SERVO_WAIT_MS = 400;

    public HingedGrabber(Servo topLeft, Servo topRight, Servo leftHinge, Servo rightHinge, Servo bottomLeft, Servo bottomRight) {
        top = new ServoPair(topLeft, topRight, 0, 1.0);
        hinge = new ServoPair(leftHinge, rightHinge, 0.1, 1.0);
        bottom = new ServoPair(bottomLeft, bottomRight, 0, 1.0);
        close();
    }

    public void close() {
        hinge.open();
        top.close();
        bottom.close();
    }

    public void open() throws InterruptedException {
        hinge.open();
        top.open();
        Thread.sleep(SERVO_WAIT_MS);
        bottom.open();
    }

    public void top_grab() throws InterruptedException {
        top.open();
        hinge.open();
        Thread.sleep(SERVO_WAIT_MS);
        hinge.close();
        Thread.sleep(SERVO_WAIT_MS);
        top.close();
        Thread.sleep(SERVO_WAIT_MS);
        hinge.open();
    }

    public void bottom_grab() {
        hinge.open();
        bottom.close();
    }

    public void bottom_open() {
        bottom.open();
    }
}
