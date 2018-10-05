package org.firstinspires.ftc.teamcode.common;

import android.opengl.Matrix;
import android.util.Pair;

import org.firstinspires.ftc.robotcore.external.matrices.GeneralMatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by efyang on 1/3/18.
 */

public class ExtendedMath {
    // ONLY WORKS FOR 3D VECTORS
    public static VectorF cross_product(VectorF a, VectorF b) {
        float[] k = a.getData();
        float[] w = b.getData();
        return new VectorF(
                k[1]*w[2] - k[2]*w[1],
                k[2]*w[0] - k[0]*w[2],
                k[0]*w[1] - k[1]*w[0]
        );
    }

    public static Pair<VectorF, MatrixF> decompose_opengl_matrix(OpenGLMatrix m) {
        // takes a 4x4 opengl transformation matrix and decomposes
        // it into its vector and matrix components
        VectorF k = new VectorF(
                m.get(0, 3),
                m.get(1, 3),
                m.get(2,3)
        );
        MatrixF w = new GeneralMatrixF(3, 3);
        for (int row = 0; row < 3; row++) {
            for (int col = 0; col < 3; col ++) {
                w.put(row, col, m.get(row, col));
            }
        }
        return Pair.create(k, w);
    }

    public static VectorF lowPass( VectorF in, VectorF out ) {
        final float ALPHA = 0.7f;
        float[] input = in.getData();
        float[] output = out.getData();
        if ( output == null ) return new VectorF(output);
        for ( int i=0; i<input.length; i++ ) {
            output[i] = output[i] + ALPHA * (input[i] - output[i]);
        }
        return new VectorF(output);
    }

    public static MatrixF get_rotation_matrix(float radians) {
        return new GeneralMatrixF(2, 2, new float[] {
                (float)Math.cos(radians), -(float)Math.sin(radians),
                (float)Math.sin(radians), (float)Math.cos(radians)
        });
    }

    public static VectorF convert_3d_to_2d(VectorF v) {
        return new VectorF(v.get(0), v.get(1));
    }

    // z-rotation is the same as normal rotation in x-y plane
    // return value in degrees
    public static float extract_z_rot(OpenGLMatrix m) {
        Orientation orientation = Orientation.getOrientation(m, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        return orientation.thirdAngle;
    }

    public static VectorF radians_to_degrees(VectorF radians) {
        VectorF ret = new VectorF(new float[radians.length()]);
        for (int i = 0; i < radians.length(); i++) {
            ret.put(i, (float)Math.toDegrees(radians.get(i)));
        }
        return ret;
    }

    public static float positive_min_degrees(float degrees) {
        degrees = degrees % 360;
        if (degrees < 0) {
            degrees += 360;
        }

        return degrees;
    }

    private static final float TWOPI = (float)(2 * Math.PI);
    public static float positive_min_radians(float radians) {
        radians = radians % TWOPI;
        if (radians < 0) {
            radians += TWOPI;
        }

        return radians;
    }

    public static float get_min_rotation_radians(float current, float target) {
        return get_min_abs_angle_radian(target - current);
    }

    public static float get_min_abs_angle_radian(float x) {
        x = positive_min_radians(x);
        float n = x - TWOPI;
        return Math.abs(x) < Math.abs(n)? x: n;
    }

    public static VectorF get_min_rotation_radians(VectorF current, VectorF target) {
        VectorF ret = new VectorF(new float[current.length()]);
        for (int i = 0; i < current.length(); i++) {
            ret.put(i, get_min_rotation_radians(current.get(i), target.get(i)));
        }
        return ret;
    }

    public static float get_min_rotation(float current, float target) {
        float pos_distance = positive_min_degrees(target - current);
        float neg_distance = pos_distance - 360;
        float dtheta;
        if (Math.abs(pos_distance) <= Math.abs(neg_distance)) {
            dtheta = pos_distance;
        } else {
            dtheta = neg_distance;
        }
        return dtheta;
    }

    // will only be necessary if diagonal movement is too much of a problem
    public static VectorF[] vector_components(VectorF v) {
        return new VectorF[] {
                new VectorF(v.get(0), 0),
                new VectorF(0, v.get(1))
        };
    }

    public static boolean angle_in_range(float x, float min, float max) {
        if (x < 0) {
            return ((x <= max) && (x >= min));
        } else {
            return angle_in_range(x - TWOPI, min, max) || ((x <= max) && (x >= min));
        }
    }

    public static boolean all_components_in_range(VectorF v, VectorF min, VectorF max) {
        // assume that all vectors are equal length
        boolean inRange = true;
        for (int i = 0; i < v.length(); i++) {
            inRange = inRange && angle_in_range(positive_min_radians(v.get(i)), min.get(i), max.get(i));
        }
        return inRange;
    }
}
