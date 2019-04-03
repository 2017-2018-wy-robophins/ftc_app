package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Timer;
import java.util.TimerTask;

public class IntegratedArmMPU6050 {
    MPU6050 internal;
    Timer timer;



    class IntegratingUpdater extends TimerTask {
        public void run() {

        }
    }

    IntegratedArmMPU6050(HardwareMap hardwareMap, String id, long updateDelay) {
        internal = hardwareMap.get(MPU6050.class, id);
        timer = new Timer();
        IntegratingUpdater updater = new IntegratingUpdater();

        timer.schedule(updater, updateDelay);
    }

    void close() {
        timer.cancel();
    }
}
