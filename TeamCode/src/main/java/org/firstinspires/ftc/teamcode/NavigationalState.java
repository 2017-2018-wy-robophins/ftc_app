package org.firstinspires.ftc.teamcode;

import android.util.Pair;

import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

import java.util.Vector;

/**
 * Created by efyang on 1/11/18.
 */

class NavigationalState {
    // 2d vector
    private VectorF position = new VectorF(new float[] {0, 0});
    // in degrees, ccw
    private float heading = 0;

    NavigationalState(){}
    NavigationalState(OpenGLMatrix m) {
        Pair<VectorF, MatrixF> decomp = ExtendedMath.decompose_opengl_matrix(m);
        this.position = decomp.first;
        this.heading = (float)Math.toDegrees(ExtendedMath.extract_z_rot(decomp.second));
    }

    VectorF get_robot_movement_vector(VectorF field_target) {
        return ExtendedMath.get_rotation_matrix((float)Math.toRadians(heading))
                .multiplied(ExtendedMath.X_REFLECT_TRANSFORM_MATRIX)
                .multiplied(field_target.subtracted(position));
    }

    float get_robot_rotation(float field_target) {
        return ExtendedMath.get_min_rotation(heading, field_target);
    }

    void move(VectorF d) {
        position.add(d);
    }

    void rotate(float theta) {
        heading = (heading + theta) % 360;
    }

    void set_position(VectorF position) {
        this.position = position;
    }

    void set_position(float xpos, float ypos) {
        this.position = new VectorF(xpos, ypos);
    }

    void set_heading(float heading) {
        this.heading = heading;
    }

    VectorF get_position() {return this.position;}
    float get_heading() {return this.heading;}

    // return how much to rotate to hit target theta
    // float execute_instruction(float x, float y, float new_theta) {
    //    move(new VectorF(dx, dy));
    //    float rot1 = new_theta
    //}
    public String toString() {
        return "Vector size: " + this.position.length() + " x: " + this.position.get(0) + " y: " + this.position.get(1) + " heading: " + this.heading;
    }
}
