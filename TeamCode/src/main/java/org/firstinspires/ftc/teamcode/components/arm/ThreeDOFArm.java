package org.firstinspires.ftc.teamcode.components.arm;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

class ThreeDOFArm implements Arm {
    private DcMotor M1;
    private DcMotor M2;
    private DcMotor M3;

    private float theta1;
    private float phi2;
    private float phi3;

    private static float L1;
    private static float L2;
    private static float L3;
    // all angles in radians
    private static float theta1Min;
    private static float theta1Max;
    private static float phi2Min;
    private static float phi2Max;
    private static float phi3Min;
    private static float phi3Max;

    ThreeDOFArm(DcMotor M1, DcMotor M2, DcMotor M3, float theta1, float phi2, float phi3, Telemetry telemetry) {
        this.M1 = M1;
        this.M2 = M2;
        this.M3 = M3;
        this.theta1 = theta1;
        this.phi2 = phi2;
        this.phi3 = phi3;
        telemetry.addLine("Instantiated Three DOF Arm");
        telemetry.addData("Initial theta1:", theta1);
        telemetry.addData("Initial phi2:", phi2);
        telemetry.addData("Initial phi3:", phi3);
        telemetry.update();
    }

    // TODO: Think about calibration?
    void setOrientation() {

    }

    void getOrientation() {

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
            return new VectorF[]{};
        }

        float w = (x*x + y*y + L1*L1 + L2*L2) / (2 * L1 * L2);
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

    private static float invkin2_getGamma(float theta2, float L1, float L2) {
        float k1 = L1 + L2 * (float)Math.cos(theta2);
        float k2 = L2 * (float)Math.sin(theta2);
        float gamma = (float)Math.atan2(k2, k1);
        return gamma;
    }
}
