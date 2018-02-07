package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

/**
 * Created by efyang on 2/6/18.
 */

class MecanumBase extends DriveBase {
    private final DcMotor NE;
    private final DcMotor NW;
    private final DcMotor SE;
    private final DcMotor SW;

    private final Telemetry telemetry;

    static final float TICKS_PER_MM = 1.4f;
    static final float TICKS_PER_DEGREE = 4.4f;

    MecanumBase(DcMotor NW, DcMotor NE, DcMotor SW, DcMotor SE, Telemetry telemetry) {
        this.NW = NW;
        this.NE = NE;
        this.SW = SW;
        this.SE = SE;
        this.telemetry = telemetry;
    }

    void move_by_vector_and_rotation(VectorF movement, float rotation, float speed, int encoder_epsilon, int timeout_ms) throws InterruptedException {
        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

        VectorF movementTickVector = movement.multiplied(TICKS_PER_MM);
        double rotationTickVector = rotation * TICKS_PER_DEGREE;

        double[] multipliers = mechanum_multipliers(
                movementTickVector.magnitude(),
                Math.atan2(movementTickVector.get(1), movementTickVector.get(0)),
                rotationTickVector);

        int NW_target = NW.getCurrentPosition() + (int)multipliers[0];
        int NE_target = NE.getCurrentPosition() + (int)multipliers[1];
        int SW_target = SW.getCurrentPosition() + (int)multipliers[2];
        int SE_target = SE.getCurrentPosition() + (int)multipliers[3];

        NW.setTargetPosition(NW_target);
        NE.setTargetPosition(NE_target);
        SW.setTargetPosition(SW_target);
        SE.setTargetPosition(SE_target);

        NW.setPower(speed);
        NE.setPower(speed);
        SW.setPower(speed);
        SE.setPower(speed);

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

            if ((Math.abs(NW_current - NW_target) < encoder_epsilon) &&
                    (Math.abs(NE_current - NE_target) < encoder_epsilon) &&
                    (Math.abs(SW_current - SW_target) < encoder_epsilon) &&
                    (Math.abs(SE_current - SE_target) < encoder_epsilon)) {
                telemetry.addLine(String.format("Move by vector %s and rotation %s interrupted: within absolute error.", movementTickVector, rotationTickVector));
                telemetry.update();
                break;
            }

            if (System.currentTimeMillis() >= next_check_timestamp) {
                if ((Math.abs(NW_current - last_NW) < RobotConstants.ENCODER_TICKS_TIMEOUT_THRESHOLD) &&
                        (Math.abs(NE_current - last_NE) < RobotConstants.ENCODER_TICKS_TIMEOUT_THRESHOLD) &&
                        (Math.abs(SW_current - last_SW) < RobotConstants.ENCODER_TICKS_TIMEOUT_THRESHOLD) &&
                        (Math.abs(SE_current - last_SE) < RobotConstants.ENCODER_TICKS_TIMEOUT_THRESHOLD)) {
                    telemetry.addLine(String.format("Move by vector %s and rotation %s interrupted: timed out.", movementTickVector, rotationTickVector));
                    telemetry.update();
                    break;
                }

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

    void move(float x, float y) {
        move_and_turn(x, y, 0);
    }

    void turn(float r) {
        move_and_turn(0, 0, r);
    }

    // x, y, r should be from -1 to 1
    void move_and_turn(float x, float y, float r) {
        setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double speed = Math.hypot(x,y) / FieldConstants.rt2;
        double angle = Math.atan2(y, x);
        double[] multipliers = mechanum_multipliers(speed, angle, r);
        telemetry.addData("mecanum multipliers", multipliers);
        NW.setPower(multipliers[0]);
        NE.setPower(multipliers[1]);
        SW.setPower(multipliers[2]);
        SE.setPower(multipliers[3]);
    }

    void stop() {
        NW.setPower(0);
        NE.setPower(0);
        SW.setPower(0);
        SE.setPower(0);
    }

    void report_encoder_ticks() {
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
    private static double[] mechanum_multipliers(double translation, double translation_angle, double rotation) {
        return new double[]{
                translation * Math.sin(translation_angle + Math.PI/4) - rotation,
                translation * Math.cos(translation_angle + Math.PI/4) + rotation,
                translation * Math.cos(translation_angle + Math.PI/4) - rotation,
                translation * Math.sin(translation_angle + Math.PI/4) + rotation
        };
    }
}
