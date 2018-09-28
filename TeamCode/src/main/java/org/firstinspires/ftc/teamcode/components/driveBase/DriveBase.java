package org.firstinspires.ftc.teamcode.components.drive_base;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

/**
 * Created by efyang on 2/6/18.
 */

public abstract class DriveBase {
    public abstract void move_by_vector_and_rotation(VectorF movement, float rotation, float speed, int encoder_epsilon, int timeout_ms) throws InterruptedException;
    // TODO: remove if unused (backup in case strafe can't be trusted)
    public abstract void rotate_and_move_only_vertical_drive(float initial_rotation, float movement, float final_rotation, float speed, int encoder_epsilon, int timeout_ms) throws InterruptedException;
    public abstract void move(float x, float y);
    public abstract void turn(float r);
    public abstract void move_and_turn(float x, float y, float r);
    public abstract void stop();
    public abstract void report_encoder_ticks();

    public void move_ms(float x, float y, int ms) throws InterruptedException {
        move(x, y);
        Thread.sleep(ms);
        stop();
    }

    public void turn_ms(float r, int ms) throws InterruptedException {
        turn(r);
        Thread.sleep(ms);
        stop();
    }

    public void move_and_turn_ms(float x, float y, float r, int ms) throws InterruptedException {
        move_and_turn(x, y, r);
        Thread.sleep(ms);
        stop();
    }
}
