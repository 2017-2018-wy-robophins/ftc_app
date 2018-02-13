package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

/**
 * Created by efyang on 2/6/18.
 */

abstract class DriveBase {
    abstract void move_by_vector_and_rotation(VectorF movement, float rotation, float speed, int encoder_epsilon, int timeout_ms) throws InterruptedException;
    // TODO: remove if unused (backup in case strafe can't be trusted)
    abstract void rotate_and_move_only_vertical_drive(float initial_rotation, float movement, float final_rotation, float speed, int encoder_epsilon, int timeout_ms) throws InterruptedException;
    abstract void move(float x, float y);
    abstract void turn(float r);
    abstract void move_and_turn(float x, float y, float r);
    abstract void stop();
    abstract void report_encoder_ticks();

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
}
