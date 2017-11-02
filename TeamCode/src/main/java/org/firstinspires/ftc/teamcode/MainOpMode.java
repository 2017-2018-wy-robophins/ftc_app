package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;




@TeleOp(name="Main Teleop", group="Robot")
public class MainOpMode extends LinearOpMode {

    //creates an instance variable fo the robot
    MainRobot robot = new MainRobot();

    //initiate all "last tick" variable for controller
    public static boolean prev_x = false;
    public static boolean prev_y = false;
    public static boolean prev_a = false;
    public static boolean prev_b = false;
    public static boolean prev_dpad_down = false;
    public static boolean prev_dpad_up = false;
    public static boolean prev_dpad_left = false;
    public static boolean prev_dpad_right = false;
    public static boolean prev_lbumper = false;
    public static boolean prev_rbumper = false;
    public static float prev_ltrigger = (float) 0;
    public static float prev_rtrigger = (float) 0;

    public static double enginePower = 1.0;
    public static double turnCoefficient = 2.0;
    public static double leftx = 0.0;
    public static double righty = 0.0;
    public static double rightx = 0.0;
    public static double lefty = 0.0;

    public static double precision = 1.0;

    public static double servoClosed = 0.3;
    public static double servoOpen = 1.0;
    @Override
    public void runOpMode() throws InterruptedException  {
        //initiate robot
        robot.init(hardwareMap);
        //initiate hardware variables
        DcMotor north = MainRobot.north;
        DcMotor west = MainRobot.west;
        DcMotor east = MainRobot.east;
        DcMotor south = MainRobot.south;
        //DcMotor pulley = MainRobot.pulley;
        Servo grab1 = MainRobot.grab1;

        telemetry.addData("say", "before opmode");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            righty = gamepad1.right_stick_y;
            rightx = gamepad1.right_stick_x;
            leftx = gamepad1.left_stick_x;
            lefty = gamepad1.left_stick_y;
            telemetry.clear();

            //hold y for precision
            if (gamepad1.y) {
                precision = 0.5;
            } else {
                precision = 1.0;
            }

            //use only protocol for the currently used joystick
            if (((Math.abs(leftx) + Math.abs(lefty))/2) >= (Math.abs(rightx) + Math.abs(righty))/2) {
                //set motor powers for transposing
                west.setPower(lefty*precision);
                east.setPower(-lefty*precision);
                north.setPower(leftx*precision);
                south.setPower(-leftx*precision);
            } else {
                //set motor powers for turning
                west.setPower(rightx*precision);
                east.setPower(rightx*precision);
                north.setPower(rightx*precision);
                south.setPower(rightx*precision);
            }

            if (gamepad1.right_bumper == true) {
                grab1.setPosition(servoClosed);
            }
            if (gamepad1.left_bumper == true) {
                grab1.setPosition(servoOpen);
            }
            if (gamepad1.dpad_up) {
                //pulley.setPower(.2);
            } else if (gamepad1.dpad_down){
                //pulley.setPower(-.2);
            } else {
                //pulley.setPower(.2);
            }

            //update telemetry
            telemetry.addData("R vertical", righty);
            telemetry.addData("L vertical",lefty);
            telemetry.addData("R horizontal", rightx);
            telemetry.addData("L horizontal", leftx);
            telemetry.addData("Engine power", enginePower);
            telemetry.addData("Servo position", grab1.getPosition());
            telemetry.addData("Precision coefficient", precision);
            telemetry.update();


            setValues();
        }
    }
    //sets 'prev values' variables to distinguish button presses tick-to-tick
    public void setValues() {
        prev_a = gamepad1.a;
        prev_b = gamepad1.b;
        prev_dpad_down = gamepad1.dpad_down;
        prev_dpad_left = gamepad1.dpad_left;
        prev_dpad_right = gamepad1.dpad_right;
        prev_dpad_up = gamepad1.dpad_up;
        prev_x = gamepad1.x;
        prev_lbumper = gamepad1.left_bumper;
        prev_rbumper = gamepad1.right_bumper;
        prev_ltrigger = gamepad1.left_trigger;
        prev_rtrigger = gamepad1.right_trigger;
        prev_y = gamepad1.y;
    }


    //a function that returns a modified value, checking if it falls within logical boundaries first
    public double incr(double value, double incr, String sign) {
        if (sign.equals("+")) {
            if (value + incr <= 1.0) {
                return value + incr;
            }
        } else if (sign.equals("-")) {
            if (value - incr >= 0.0) {
                return value - incr;
            }
        }
        return value;
    }

}