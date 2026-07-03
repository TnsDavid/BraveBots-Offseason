package org.firstinspires.ftc.teamcode.util.math;

public class LowPassFilter {
    public double alpha;
    public double y;

    public LowPassFilter(double cutOffFrequency, double dt, double initialValue) {
        double rc = 1.0 / (2.0 * Math.PI * cutOffFrequency);
        this.alpha = dt / (dt + rc);
        this.y = initialValue;
    }

    public LowPassFilter(double alpha, double initialValue) {
        this.alpha = alpha;
        this.y = initialValue;
    }

    public double update(double x) {
        y = alpha * x + (1.0 - alpha) * y;
        return y;
    }
}

