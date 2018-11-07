package org.firstinspires.ftc.teamcode.components.visionProcessor;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.teamcode.common.SamplingConfiguration;

public interface VisionProcessor {
    OpenGLMatrix getCurrentPosition();
    SamplingConfiguration getSamplingConfiguration();
}
