package org.firstinspires.ftc.teamcode.components.grabber;

/**
 * Created by efyang on 2/13/18.
 */

public interface Grabber {
    void open() throws InterruptedException;
    void close() throws InterruptedException;
    void top_grab() throws InterruptedException;
    void bottom_grab() throws InterruptedException;
}
