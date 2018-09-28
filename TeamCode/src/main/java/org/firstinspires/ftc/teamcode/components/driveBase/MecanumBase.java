package org.firstinspires.ftc.teamcode.components.driveBase;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.common.RobotConstants;
import org.firstinspires.ftc.teamcode.components.inertialSensor.InertialSensor;

/**
 * Created by efyang on 2/6/18.
 */

public class MecanumBase extends DriveBase {
    private final DcMotor NE;
    private final DcMotor NW;
    private final DcMotor SE;
    private final DcMotor SW;

    private final Telemetry telemetry;

    static final float TICKS_PER_MM = 2f;
    static final float TICKS_PER_DEGREE = 12.5f;

    public MecanumBase(DcMotor NW, DcMotor NE, DcMotor SW, DcMotor SE, Telemetry telemetry) {
        this.NW = NW;
        this.NE = NE;
        this.SW = SW;
        this.SE = SE;

        // reset motor encoders
        NW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        NE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        NW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        NE.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SE.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // set them to brake when 0 power
        NW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SW.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        NE.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SE.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // set motor directions
        NW.setDirection(DcMotor.Direction.FORWARD);
        SW.setDirection(DcMotor.Direction.FORWARD);
        NE.setDirection(DcMotor.Direction.REVERSE);
        SE.setDirection(DcMotor.Direction.REVERSE);

        this.telemetry = telemetry;
    }

    public void move_by_vector_and_rotation(VectorF movement, float rotation, float speed, int encoder_epsilon, int timeout_ms) throws InterruptedException {
        VectorF movementTickVector = movement.multiplied(TICKS_PER_MM);
        double rotationTickVector = rotation * TICKS_PER_DEGREE;

        double[] multipliers = mecanum_multipliers(
                movementTickVector.magnitude(),
                Math.atan2(movementTickVector.get(1), movementTickVector.get(0)),
                rotationTickVector);

        set_encoder_dx(
                (int)multipliers[0],
                (int)multipliers[1],
                (int)multipliers[2],
                (int)multipliers[3],
                speed,
                encoder_epsilon,
                timeout_ms,
                String.format("Move by vector %s and rotation %s interrupted: within absolute error.", movement, rotation),
                String.format("Move by vector %s and rotation %s interrupted: timed out.", movement, rotation)
                );
    }

    // rotate, then move, then rotate
    public void rotate_and_move_only_vertical_drive(float initial_rotation, float movement, float final_rotation, float speed, int encoder_epsilon, int timeout_ms) throws InterruptedException {
        rotate(initial_rotation, speed, encoder_epsilon, timeout_ms);
        move_vertical(movement, speed, encoder_epsilon, timeout_ms);
        rotate(final_rotation, speed, encoder_epsilon, timeout_ms);
    }

    private void move_vertical(float movement, float speed, int encoder_epsilon, int timeout_ms) throws InterruptedException {
        int movement_ticks = (int)(movement * TICKS_PER_MM);

        set_encoder_dx(
                movement_ticks,
                movement_ticks,
                movement_ticks,
                movement_ticks,
                speed,
                encoder_epsilon,
                timeout_ms,
                String.format("Movement by amount %s interrupted: within absolute error", movement_ticks),
                String.format("Movement by amount %s interrupted: timed out.", movement_ticks)
                );
    }

    private void rotate(float rotation, float speed, int encoder_epsilon, int timeout_ms) throws InterruptedException {
        int rotation_ticks = (int)(rotation * TICKS_PER_DEGREE);

        set_encoder_dx(
                -rotation_ticks,
                rotation_ticks,
                -rotation_ticks,
                rotation_ticks,
                speed,
                encoder_epsilon,
                timeout_ms,
                String.format("Rotation by amount %s interrupted: within absolute error", rotation_ticks),
                String.format("Rotation by amount %s interrupted: timed out.", rotation_ticks)
                );
    }

    private void set_encoder_dx(int dNW, int dNE, int dSW, int dSE, float speed, int encoder_epsilon, int timeout_ms, String epsilon_error, String timeout_error) throws InterruptedException {
        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

        // get the target encoder values
        int NW_target = NW.getCurrentPosition() + dNW;
        int NE_target = NE.getCurrentPosition() + dNE;
        int SW_target = SW.getCurrentPosition() + dSW;
        int SE_target = SE.getCurrentPosition() + dSE;

        // set the motors to the target
        NW.setTargetPosition(NW_target);
        NE.setTargetPosition(NE_target);
        SW.setTargetPosition(SW_target);
        SE.setTargetPosition(SE_target);

        // set the speed to move at (setPower runs by speed when using RUN_TO_POSITION)
        NW.setPower(speed);
        NE.setPower(speed);
        SW.setPower(speed);
        SE.setPower(speed);

        // setup the last seen encoder values
        // these get used in conjuction with a timer to check if the robot has been blocked by something
        int last_NW = NW.getCurrentPosition();
        int last_NE = NE.getCurrentPosition();
        int last_SW = SW.getCurrentPosition();
        int last_SE = SE.getCurrentPosition();
        long next_check_timestamp = System.currentTimeMillis() + timeout_ms;

        while (NW.isBusy() || NE.isBusy() || SW.isBusy() || SE.isBusy()) {
            int NW_current = NW.getCurrentPosition();
            int NE_current = NE.getCurrentPosition();
            int SW_current = SW.getCurrentPosition();
            int SE_current = SE.getCurrentPosition();
            telemetry.addData("NW", NW_current);
            telemetry.addData("NE", NE_current);
            telemetry.addData("SW", SW_current);
            telemetry.addData("SE", SE_current);
            telemetry.addData("NW target", NW_target);
            telemetry.addData("NE target", NE_target);
            telemetry.addData("SW target", SW_target);
            telemetry.addData("SE target", SE_target);
            telemetry.update();

            // when the robot is within the epsilon, stop early
            if ((Math.abs(NW_current - NW_target) < encoder_epsilon) &&
                    (Math.abs(NE_current - NE_target) < encoder_epsilon) &&
                    (Math.abs(SW_current - SW_target) < encoder_epsilon) &&
                    (Math.abs(SE_current - SE_target) < encoder_epsilon)) {
                telemetry.addLine(epsilon_error);
                telemetry.update();
                break;
            }

            if (System.currentTimeMillis() >= next_check_timestamp) {
                // when the robot hasn't moved much for the past <time>, stop early
                if ((Math.abs(NW_current - last_NW) < RobotConstants.ENCODER_TICKS_TIMEOUT_THRESHOLD) &&
                        (Math.abs(NE_current - last_NE) < RobotConstants.ENCODER_TICKS_TIMEOUT_THRESHOLD) &&
                        (Math.abs(SW_current - last_SW) < RobotConstants.ENCODER_TICKS_TIMEOUT_THRESHOLD) &&
                        (Math.abs(SE_current - last_SE) < RobotConstants.ENCODER_TICKS_TIMEOUT_THRESHOLD)) {
                    telemetry.addLine(timeout_error);
                    telemetry.update();
                    break;
                }

                // update the timestamps and values
                last_NW = NW_current;
                last_NE = NE_current;
                last_SW = SW_current;
                last_SE = SE_current;
                next_check_timestamp = System.currentTimeMillis() + timeout_ms;
            }

            Thread.sleep(5);
        }

        stop();
    }

    public void move(float x, float y) {
        move_and_turn(x, y, 0);
    }

    public void turn(float r) {
        move_and_turn(0, 0, r);
    }

    // TODO: unimplemented
    public void imu_turn(float r, InertialSensor imu) {

    }

    // x, y, r should be from -1 to 1
    public void move_and_turn(float x, float y, float r) {
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double speed = Math.hypot(x,y);
        double angle = Math.atan2(y, x);
        double[] multipliers = mecanum_multipliers_main(speed, angle, r);
        telemetry.addData("NW Power", multipliers[0]);
        telemetry.addData("NE Power", multipliers[1]);
        telemetry.addData("SW Power", multipliers[2]);
        telemetry.addData("SE Power", multipliers[3]);
        NW.setPower(multipliers[0]);
        NE.setPower(multipliers[1]);
        SW.setPower(multipliers[2]);
        SE.setPower(multipliers[3]);
    }

    public void stop() {
        NW.setPower(0);
        NE.setPower(0);
        SW.setPower(0);
        SE.setPower(0);
    }

    public void report_encoder_ticks() {
        telemetry.addData("NW ticks", NW.getCurrentPosition());
        telemetry.addData("NE ticks", NE.getCurrentPosition());
        telemetry.addData("SW ticks", SW.getCurrentPosition());
        telemetry.addData("SE ticks", SE.getCurrentPosition());
    }

    private void setMotorMode(DcMotor.RunMode mode) {
        NW.setMode(mode);
        NE.setMode(mode);
        SW.setMode(mode);
        SE.setMode(mode);
    }

    // http://thinktank.wpi.edu/resources/346/ControllingMecanumDrive.pdf
    // return [NW, NE, SW, SE]
    // angle should be in radians
    // adjusted version for the main driving opmode
    private static double[] mecanum_multipliers_main(double translation, double translation_angle, double rotation) {
        return new double[]{
                1.5 * translation * Math.sin(-translation_angle + 3*Math.PI/4) - rotation,
                1.5 * translation * Math.cos(-translation_angle + 3*Math.PI/4) + rotation,
                1.5 * translation * Math.cos(-translation_angle + 3*Math.PI/4) - rotation,
                1.5 * translation * Math.sin(-translation_angle + 3*Math.PI/4) + rotation
        };
    }

    // normal version
    private static double[] mecanum_multipliers(double translation, double translation_angle, double rotation) {
        return new double[]{
                translation * Math.sin(-translation_angle + 3*Math.PI/4) - rotation,
                translation * Math.cos(-translation_angle + 3*Math.PI/4) + rotation,
                translation * Math.cos(-translation_angle + 3*Math.PI/4) - rotation,
                translation * Math.sin(-translation_angle + 3*Math.PI/4) + rotation
        };
    }
}
