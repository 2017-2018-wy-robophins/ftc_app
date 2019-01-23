package org.firstinspires.ftc.teamcode.components.driveBase;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.components.inertialSensor.InertialSensor;

/**
 * Created by efyang on 2/6/18.
 */

//Provides standardized functions for any drive base.
public abstract class DriveBase {
    public abstract void imu_turn(float r, InertialSensor imu);
    public abstract void imu_forward_move(float x, InertialSensor imu);
    public abstract void direct_move_and_turn(float x, float r);
    public abstract void stop();
    public abstract void report_encoder_ticks();

    public void move_ms(float x, int ms) throws InterruptedException {
        direct_move_and_turn(x, 0);
        Thread.sleep(ms);
        stop();
    }

    public void turn_ms(float r, int ms) throws InterruptedException {
        direct_move_and_turn(0, r);
        Thread.sleep(ms);
        stop();
    }

    public void move_and_turn_ms(float x, float r, int ms) throws InterruptedException {
        direct_move_and_turn(x, r);
        Thread.sleep(ms);
        stop();
    }
}
