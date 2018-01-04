package org.firstinspires.ftc.teamcode;

import android.opengl.Matrix;
import android.util.Pair;

import org.firstinspires.ftc.robotcore.external.matrices.GeneralMatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

/**
 * Created by efyang on 1/3/18.
 */

class ExtendedMath {
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

    static final float ALPHA = 0.7f;
    public static VectorF lowPass( VectorF in, VectorF out ) {
        float[] input = in.getData();
        float[] output = out.getData();
        if ( output == null ) return new VectorF(output);
        for ( int i=0; i<input.length; i++ ) {
            output[i] = output[i] + ALPHA * (input[i] - output[i]);
        }
        return new VectorF(output);
    }
}
