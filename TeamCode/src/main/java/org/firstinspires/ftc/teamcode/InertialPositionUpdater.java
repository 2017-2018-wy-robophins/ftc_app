package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Handler;
import android.os.HandlerThread;
import android.util.Pair;

import com.sun.tools.javac.util.Position;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

/**
 * Created by efyang on 1/3/18.
 */
// based on https://stackoverflow.com/questions/20048646/accesing-android-sensors-in-non-activity-class
class InertialPositionUpdater implements SensorEventListener {
    Context appContext;
    private VectorF heading = new VectorF(0, 0,0);
    private VectorF angular_velocity = new VectorF(0, 0, 0);
    private VectorF position = new VectorF(0, 0, 0);
    private VectorF velocity = new VectorF(0, 0, 0);

    private VectorF acceleration = new VectorF(0,0,0);
    private VectorF omega = new VectorF(0,0,0);

    private VectorF phone_relative_position;
    private MatrixF rotation_matrix;

    private static final float NS_TO_S = 1.0f / 1000_000_000.0f;
    private float previous_timestamp = 0;
    private final SensorManager mSensorManager;
    private HandlerThread mSensorThread;
    private Handler mSensorHandler;
    private final Sensor mAccelerometer;
    private final Sensor mGyroscope;

    public InertialPositionUpdater(OpenGLMatrix transformation_matrix, Context context) {
        Pair<VectorF, MatrixF> decomposed = ExtendedMath.decompose_opengl_matrix(transformation_matrix);
        phone_relative_position = decomposed.first;
        rotation_matrix = decomposed.second;

        appContext = context;

        mSensorThread = new HandlerThread("Sensor thread", Thread.MAX_PRIORITY);
        mSensorThread.start();
        mSensorHandler = new Handler(mSensorThread.getLooper());
        mSensorManager = (SensorManager) appContext.getSystemService(Context.SENSOR_SERVICE);

        // mSensorManager = (SensorManager) this.getSystemService(Context.SENSOR_SERVICE);
        // LINEAR_ACCELERATION for without gravity
        mAccelerometer = mSensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION);
        mGyroscope = mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);

        mSensorManager.registerListener(this, mAccelerometer, SensorManager.SENSOR_DELAY_GAME, mSensorHandler);
        mSensorManager.registerListener(this, mGyroscope, SensorManager.SENSOR_DELAY_GAME, mSensorHandler);
    }

    public void onAccuracyChanged(Sensor sensor, int accuracy) {}

    public void onSensorChanged(SensorEvent event) {
        Sensor sensor = event.sensor;
        boolean event_happened = false;
        switch (sensor.getType()) {
            case Sensor.TYPE_LINEAR_ACCELERATION:
                // accelerometer
                // set everything to 0 on the first one
                if (previous_timestamp == 0) {
                    previous_timestamp = event.timestamp;
                } else {
                    event_happened = true;
                    float dT = (event.timestamp - previous_timestamp) * NS_TO_S;
                    VectorF a = new VectorF(event.values);
                    acceleration = ExtendedMath.lowPass(acceleration, a);

                    VectorF dV = rotation_matrix.multiplied(
                            acceleration
                            .multiplied(dT)
                            .subtracted(ExtendedMath.cross_product(angular_velocity, phone_relative_position))
                    );
                    velocity.add(dV);
                    VectorF dX = velocity.multiplied(dT);
                    position.add(dX);
                }
                break;

            case Sensor.TYPE_GYROSCOPE:
                // gyroscope
                // set everything to 0 on the first one
                if (previous_timestamp == 0) {
                    previous_timestamp = event.timestamp;
                } else {
                    event_happened = true;
                    float dT = (event.timestamp - previous_timestamp) * NS_TO_S;
                    VectorF omega_r = new VectorF(event.values);
                    omega = ExtendedMath.lowPass(omega_r, omega);
                    angular_velocity = omega;
                    VectorF dtheta = omega.multiplied(dT);
                    heading.add(dtheta);
                }
                break;
        }
        if (event_happened) {
            previous_timestamp = event.timestamp;
        }
    }

    VectorF getVelocity() {
        return velocity;
    }

    VectorF getPosition() {
        return position;
    }

    VectorF getHeading() {
        return heading;
    }

    VectorF getAcceleration() {
        return acceleration;
    }

    VectorF getOmega() {
        return omega;
    }

    float getPrevious_timestamp() {
        return previous_timestamp * NS_TO_S;
    }
}
