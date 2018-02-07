package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

/**
 * Created by efyang on 2/6/18.
 */

abstract class DriveBase {
    abstract void move_by_vector_and_rotation(VectorF movement, float rotation, float speed, int encoder_epsilon, int timeout_ms) throws InterruptedException;
    abstract void move(float x, float y);
    abstract void turn(float r);
    abstract void move_and_turn(float x, float y, float r);
    abstract void stop();

    void move_ms(float x, float y, int ms) throws InterruptedException {
        move(x, y);
        Thread.sleep(ms);
        stop();
    }

    void turn_ms(float r, int ms) throws InterruptedException {
        turn(r);
        Thread.sleep(ms);
        stop();
    }

    void move_and_turn_ms(float x, float y, float r, int ms) throws InterruptedException {
        move_and_turn(x, y, r);
        Thread.sleep(ms);
        stop();
    }

    abstract void report_encoder_ticks();
}
