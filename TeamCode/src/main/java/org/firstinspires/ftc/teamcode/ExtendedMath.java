package org.firstinspires.ftc.teamcode;

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

class ExtendedMath {
    // ONLY WORKS FOR 3D VECTORS
    static VectorF cross_product(VectorF a, VectorF b) {
        float[] k = a.getData();
        float[] w = b.getData();
        return new VectorF(
                k[1]*w[2] - k[2]*w[1],
                k[2]*w[0] - k[0]*w[2],
                k[0]*w[1] - k[1]*w[0]
        );
    }

    static Pair<VectorF, MatrixF> decompose_opengl_matrix(OpenGLMatrix m) {
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

    static VectorF lowPass( VectorF in, VectorF out ) {
        final float ALPHA = 0.7f;
        float[] input = in.getData();
        float[] output = out.getData();
        if ( output == null ) return new VectorF(output);
        for ( int i=0; i<input.length; i++ ) {
            output[i] = output[i] + ALPHA * (input[i] - output[i]);
        }
        return new VectorF(output);
    }

    static MatrixF get_rotation_matrix(float radians) {
        return new GeneralMatrixF(2, 2, new float[] {
                (float)Math.cos(radians), -(float)Math.sin(radians),
                (float)Math.sin(radians), (float)Math.cos(radians)
        });
    }

    static VectorF convert_3d_to_2d(VectorF v) {
        return new VectorF(v.get(0), v.get(1));
    }

    // z-rotation is the same as normal rotation in x-y plane
    // return value in degrees
    static float extract_z_rot(OpenGLMatrix m) {
        Orientation orientation = Orientation.getOrientation(m, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        return orientation.thirdAngle;
    }

    private static float positive_min_degrees(float degrees) {
        degrees = degrees % 360;
        if (degrees < 0) {
            degrees += 360;
        }

        return degrees;
    }

    static float get_min_rotation(float current, float target) {
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
    static VectorF[] vector_components(VectorF v) {
        return new VectorF[] {
                new VectorF(v.get(0), 0),
                new VectorF(0, v.get(1))
        };
    }
}
