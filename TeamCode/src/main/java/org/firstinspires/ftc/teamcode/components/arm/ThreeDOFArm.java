package org.firstinspires.ftc.teamcode.components.arm;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.common.ExtendedMath;

import java.util.Arrays;

public class ThreeDOFArm implements Arm {
    private static Telemetry telemetry;
    private static float L1 = 304;
    private static float L2 = 292;
    private static float L3 = 203.2f;
    // all angles in radians
    private static float theta1Min = (float)Math.toRadians(0);
    private static float theta1Max = (float)Math.toRadians(180);
    private static float phi2Min = (float)Math.toRadians(-30);
    private static float phi2Max = (float)Math.toRadians(150);
    private static float phi3Min = (float)Math.toRadians(0);
    private static float phi3Max = (float)Math.toRadians(360);

    private static final float M1_RADIAN_TO_COUNT_RATIO = 1265/(float)Math.toRadians(90);
    private static final float M2_RADIAN_TO_COUNT_RATIO = 100/(float)Math.toRadians(90);
    private static final float M3_RADIAN_TO_COUNT_RATIO = 100/(float)Math.toRadians(90);

    // Motor Tolerances
    private static final float TARGET_RADIAN_TOLERANCE = (float)Math.toRadians(0.5);
    private static final int M1_TOLERANCE = (int)Math.abs(TARGET_RADIAN_TOLERANCE * M1_RADIAN_TO_COUNT_RATIO);
    private static final int M2_TOLERANCE = (int)Math.abs(TARGET_RADIAN_TOLERANCE * M2_RADIAN_TO_COUNT_RATIO);
    private static final int M3_TOLERANCE = (int)Math.abs(TARGET_RADIAN_TOLERANCE * M3_RADIAN_TO_COUNT_RATIO);
    private static final float SPEED = 0.2f;

    private static VectorF angleMin = new VectorF(theta1Min, phi2Min, phi3Min);
    private static VectorF angleMax = new VectorF(theta1Max, phi2Max, phi3Max);

    private DcMotorEx M1;
    private DcMotorEx M2;
    private DcMotorEx M3;

    private float theta1i;
    private float phi2i;
    private float phi3i;
    private VectorF current;


    public ThreeDOFArm(DcMotorEx M1, DcMotorEx M2, DcMotorEx M3, float theta1, float phi2, float phi3, Telemetry telemetry) {
        this.M1 = M1;
        this.M2 = M2;
        this.M3 = M3;
        hardwareInit();

        this.telemetry = telemetry;
        this.theta1i = theta1;
        this.phi2i = phi2;
        this.phi3i = phi3;
        this.current = new VectorF(theta1, phi2, phi3);
        telemetry.addLine("Instantiated Three DOF Arm");
        telemetry.addData("Initial theta1:", theta1);
        telemetry.addData("Initial phi2:", phi2);
        telemetry.addData("Initial phi3:", phi3);
        telemetry.update();
    }

    private void hardwareInit() {
        setMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        M1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        M3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        M1.setDirection(DcMotorSimple.Direction.REVERSE);
        M2.setDirection(DcMotorSimple.Direction.FORWARD);
        M3.setDirection(DcMotorSimple.Direction.FORWARD);

        M1.setTargetPositionTolerance(M1_TOLERANCE);
        M2.setTargetPositionTolerance(M2_TOLERANCE);
        M3.setTargetPositionTolerance(M3_TOLERANCE);
    }

private VectorF getNewOrientation(VectorF target) {
        VectorF[] possible = invkin3(target, L1, L2, L3);
        telemetry.addData("Possible news", Arrays.toString(possible));
        telemetry.addData("Range min", angleMin);
        telemetry.addData("Range max", angleMax);
        VectorF[] filtered = Arrays.stream(possible)
                .filter(c -> ExtendedMath.all_components_in_range(c, angleMin, angleMax))
                .toArray(VectorF[]::new);
        if (filtered.length == 0) {
            return null;
        } else {
            Arrays.sort(filtered,
                    (a, b) -> -1 * Float.compare(a.subtracted(current).magnitude(), b.subtracted(current).magnitude())
            );
            return filtered[0];
        }
    }

    // boolean denotes whether changed
    public boolean goToTarget(VectorF target) {
        updateOrientation();
        VectorF newOrientation = getNewOrientation(target);
        if (newOrientation == null) {
            return false;
        } else {
            telemetry.addData("possible new", ExtendedMath.radians_to_degrees(newOrientation));
            // find the fastest way to get from current to new (shortest abs)
            VectorF change = ExtendedMath.get_min_rotation_radians(current, newOrientation);
            telemetry.addData("change", ExtendedMath.radians_to_degrees(change));
            float deltaTheta1 = change.get(0);
            float deltaPhi2 = change.get(1);
            float deltaPhi3 = change.get(2);

            // now set the encoders
            setEncoderDRadian(deltaTheta1, deltaPhi2, deltaPhi3);
            current = newOrientation;
            return true;
        }
    }

    private void setMotorMode(DcMotor.RunMode mode) {
        M1.setMode(mode);
        M2.setMode(mode);
        M3.setMode(mode);
    }

    private void setEncoderDRadian(float a, float b, float c) {
        setEncoderDx((int)(a * M1_RADIAN_TO_COUNT_RATIO), (int)(b * M2_RADIAN_TO_COUNT_RATIO), (int)(c * M3_RADIAN_TO_COUNT_RATIO));
    }

    private void setEncoderDx(int dM1, int dM2, int dM3) {
        int M1_TARGET = M1.getCurrentPosition() + dM1;
        int M2_TARGET = M2.getCurrentPosition() + dM2;
        int M3_TARGET = M3.getCurrentPosition() + dM3;

        M1.setTargetPosition(M1_TARGET);
        M2.setTargetPosition(M2_TARGET);
        M3.setTargetPosition(M3_TARGET);

        M1.setPower(SPEED);
        M2.setPower(SPEED);
        M3.setPower(SPEED);
    }

    public VectorF getOrientation() {
        updateOrientation();
        return current;
    }

    private void updateOrientation() {
        float theta1 = theta1i + M1.getCurrentPosition() / M1_RADIAN_TO_COUNT_RATIO;
        float phi2 = phi2i + M2.getCurrentPosition() / M2_RADIAN_TO_COUNT_RATIO;
        float phi3 = phi3i + M3.getCurrentPosition() / M3_RADIAN_TO_COUNT_RATIO;
        current = new VectorF(theta1, phi2, phi3);
    }

    public VectorF getCartesianPosition() {
        VectorF angles = getOrientation();
        float theta1 = angles.get(0);
        float phi2 = angles.get(1);
        float phi3 = angles.get(2);
        float r = (float)(L1 + L2 * Math.cos(phi2) + L3 * Math.cos(phi2 + phi3));
        return new VectorF(
                (float)(r * Math.cos(theta1)),
                (float)(r * Math.sin(theta1)),
                (float)(L2 * Math.sin(phi2) + L3 * Math.sin(phi2 + phi3))
        );
    }

    // returns [VectorF(theta1, phi2, phi3)]
    private static VectorF[] invkin3(VectorF target, float L1, float L2, float L3) {
        float x = target.get(0);
        float y = target.get(1);
        float z = target.get(2);

        float theta1 = (float)Math.atan2(y, x);
        float xPrime = x - L1 * (float)Math.cos(theta1);
        float yPrime = y - L2 * (float)Math.sin(theta1);

        VectorF[] invkin2_solns = invkin2(new VectorF((float)Math.sqrt(xPrime*xPrime + yPrime*yPrime), z), L2, L3);
        telemetry.addData("invkin2", Arrays.toString(invkin2_solns));

        VectorF[] solns = new VectorF[invkin2_solns.length];
        for (int i = 0; i < invkin2_solns.length; i++) {
            VectorF invkin2_soln = invkin2_solns[i];
            solns[i] = new VectorF(theta1, invkin2_soln.get(0), invkin2_soln.get(1));
        }
        return solns;
    }

    // based on https://ashwinnarayan.blogspot.com/2014/07/inverse-kinematics-for-2dof-arm.html
    // returns [VectorF(theta1, theta2)]
    private static VectorF[] invkin2(VectorF target, float L1, float L2) {
        float x = target.get(0);
        float y = target.get(1);

        // infinite/no solutions or out of range
        if ((x == 0 && y == 0) || (target.magnitude() > L1 + L2)) {
            return new VectorF[0];
        } else {
            float w = (x*x + y*y - L1*L1 - L2*L2) / (2 * L1 * L2);
            // positive and negative solution
            float theta2p = (float)Math.atan2(Math.sqrt(1 - w*w), w);
            float theta2n = -1 * theta2p;

            float theta1p = invkin2_getGamma(theta2p, L1, L2);
            float theta1n = invkin2_getGamma(theta2n, L1, L2);

            // TODO: handle single solution case?
            return new VectorF[]{
                    new VectorF(theta1p, theta2p),
                    new VectorF(theta1n, theta2n)
            };
        }
    }

    private static float invkin2_getGamma(float theta2, float L1, float L2) {
        float k1 = L1 + L2 * (float)Math.cos(theta2);
        float k2 = L2 * (float)Math.sin(theta2);
        return (float)Math.atan2(k2, k1);
    }
}
