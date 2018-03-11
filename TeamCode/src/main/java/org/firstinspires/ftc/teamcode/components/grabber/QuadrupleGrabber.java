package org.firstinspires.ftc.teamcode.components.grabber;

import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by efyang on 2/6/18.
 */

public class QuadrupleGrabber implements Grabber {
    private final ServoPair top;
    private final ServoPair bottom;
    private final int SERVO_WAIT_MS = 300;

    public QuadrupleGrabber(Servo topLeft, Servo topRight, Servo bottomLeft, Servo bottomRight) {
        top = new ServoPair(topLeft, topRight, 0, 1);
        bottom = new ServoPair(bottomLeft, bottomRight, 0, 1);
    }

    public void close() {
        top.close();
        bottom.close();
    }

    public void open() throws InterruptedException {
        top.open();
        Thread.sleep(SERVO_WAIT_MS);
        bottom.open();
    }

    public void top_grab() throws InterruptedException {
        top.close();
    }

    public void bottom_grab() throws InterruptedException {
        bottom.close();
    }

    public void bottom_open() throws InterruptedException {
        bottom.open();
    }
}
