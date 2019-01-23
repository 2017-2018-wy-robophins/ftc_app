package org.firstinspires.ftc.teamcode.autonomous;

import android.util.Pair;

import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.common.ExtendedMath;
import org.firstinspires.ftc.teamcode.common.SamplingConfiguration;
import org.firstinspires.ftc.teamcode.common.StartLocation;

/**
 * Created by efyang on 1/11/18.
 */

public class NavigationalState {
    public StartLocation startLocation;
    // 2d vector
    private VectorF position = new VectorF(new float[] {0, 0});
    // in degrees, ccw
    private float heading = 0;

    private SamplingConfiguration detectedSample = SamplingConfiguration.RIGHT;

    //A type that decomposes location matrices to an absolute angle and position.
    public NavigationalState(StartLocation startLocation){
        this.startLocation = startLocation;
    }
    public NavigationalState(OpenGLMatrix m, StartLocation startLocation) {
        this.startLocation = startLocation;
        Pair<VectorF, MatrixF> decomp = ExtendedMath.decompose_opengl_matrix(m);
        this.position = ExtendedMath.convert_3d_to_2d(decomp.first);
        this.heading = ExtendedMath.extract_z_rot(m);
    }

    // return robot-relative movement vector to get to target
    public VectorF get_robot_movement_vector(VectorF field_target) {
        return ExtendedMath.get_rotation_matrix(-(float)Math.toRadians(heading - 90))
                .multiplied(field_target.subtracted(position));
    }

    //Returns the distance from the robot to any position in the field.
    public float get_distance(VectorF field_target) {
        return field_target.subtracted(position).magnitude();
    }

    // return how much to rotate to get to target
    public float get_robot_rotation(float field_target) {
        return ExtendedMath.get_min_rotation(heading, field_target);
    }

    //Returns the orientation of sampling objects.
    public SamplingConfiguration get_sampling_configuration() {
        return this.detectedSample;
    }


    //A series of functions to record movement of the robot, called during movement commands.

    public void move(VectorF d) {
        position.add(d);
    }

    public void rotate(float theta) {
        heading = (heading + theta) % 360;
    }

    public void set_position(VectorF position) {
        this.position = position;
    }

    public void set_position(float xpos, float ypos) {
        this.position = new VectorF(xpos, ypos);
    }

    public void set_heading(float heading) {
        this.heading = heading;
    }

    public void set_sampling_configuration(SamplingConfiguration samplingConfiguration) {
        this.detectedSample = samplingConfiguration;
    }

    public VectorF get_position() {return this.position;}
    public float get_heading() {return this.heading;}

    public String toString() {
        return "Position: " + this.position + " | Heading: " + this.heading;
    }
}
