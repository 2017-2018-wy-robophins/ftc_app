package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DistanceSensor;


class MainRobot {
    //define all variables used
    DcMotor NE;
    DcMotor NW;
    DcMotor SE;
    DcMotor SW;
    DcMotor arm;
    Servo grab1;
    Servo grab2;
    Servo colorSensorServo;
    DistanceSensor colorDistanceSensor;
    // ColorSensor colorSensor;


    //runs on press of the "init" button. Maps engines from the robot to variables,
    void init(HardwareMap HM) {
        NE = HM.dcMotor.get("NE");
        NW = HM.dcMotor.get("NW");
        SE = HM.dcMotor.get("SE");
        SW = HM.dcMotor.get("SW");
        arm = HM.dcMotor.get("arm");
        grab1 =  HM.servo.get("grab1");
        grab2 =  HM.servo.get("grab2");
        colorSensorServo = HM.servo.get("colorSensorServo");
        colorDistanceSensor = HM.get(DistanceSensor.class, "colorDistanceSensor");
        NW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        NE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SW.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        NW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        NE.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SE.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setDirection(DcMotor.Direction.REVERSE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    private static final double servoClosed = 1.0;
    private static final double servoOpen = 0;

    void closeServo() {
        grab1.setPosition(servoClosed);
        grab2.setPosition(servoOpen);
    }
    void openServo() throws InterruptedException {
        grab1.setPosition(servoOpen);
        Thread.sleep(500);
        grab2.setPosition(servoClosed);
        Thread.sleep(500);
    }

    private void move_arm_ticks(int ticks, float power, int timeout) throws InterruptedException {
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int arm_target = arm.getCurrentPosition() + ticks;
        arm.setTargetPosition(arm_target);
        arm.setPower(power);
        int last_arm = arm.getCurrentPosition();
        long next_check_timestamp = System.currentTimeMillis() + timeout;


        while (arm.isBusy()) {
            int arm_current = arm.getCurrentPosition();
            if (Math.abs(arm_current - arm_target) < RobotConstants.TICK_ALLOWED_ABS_ERROR) {
                break;
            }

            if (System.currentTimeMillis() >= next_check_timestamp) {
                if ((Math.abs(arm_current - last_arm) < RobotConstants.ENCODER_TICKS_TIMEOUT_THRESHOLD)) {
                    break;
                }
                last_arm = arm.getCurrentPosition();
                next_check_timestamp = System.currentTimeMillis() + timeout;
            }

            Thread.sleep(5);
        }
        arm.setPower(0);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    void move_arm_up(float power) throws InterruptedException {
        move_arm_ticks(RobotConstants.ARM_MOVEMENT_TICKS, power, RobotConstants.MOTOR_TIMEOUT_MILLIS);
    }

    void move_arm_down(float power) throws InterruptedException {
        move_arm_ticks(-RobotConstants.ARM_MOVEMENT_TICKS, power, RobotConstants.MOTOR_TIMEOUT_MILLIS);
    }

    void stop() {
        NW.setPower(0);
        NE.setPower(0);
        SW.setPower(0);
        SE.setPower(0);
    }
}