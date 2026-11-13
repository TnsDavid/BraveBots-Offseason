package org.firstinspires.ftc.teamcode.util.math;


public class PDSFController extends PIDFController {
    public double kS, ksOutput, sign;
    public PDSFController(double kp, double kd, double ks, double kf) {
        super(kp, 0, kd, kf);
        this.kS = ks;
    }

    public PDSFController(double kp, double ki, double kd, double kf, double ks) {
        super(kp, ki, kd, kf);
        this.kS = ks;
    }


    @Override
    public double calculate(double current, double target) {

        double err = target - current;
        sign = (err >= 0) ? 1 : -1; //never null
        ksOutput = kS * sign;

        return super.calculate(current, target) + ksOutput;
    }

    public double calculate(double error) {
        ksOutput = kS;
        return super.calculate(error) + ksOutput;
    }

    public void setCoefficients(double kP, double kD, double kF, double kS) {
        this.kS = kS;
        super.setPIDF(kP, 0, kD, kF);
    }

    public void setCoefficients(PDSFCoefficients coefficients) {
        this.kS = coefficients.s;
        super.setPIDF(kP, 0, kD, kF);

    }
}