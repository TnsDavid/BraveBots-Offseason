package org.firstinspires.ftc.teamcode.madoarerauinpula.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.madoarerauinpula.Localizer;
import org.firstinspires.ftc.teamcode.robot.globals.Alliance;
import org.firstinspires.ftc.teamcode.robot.globals.Robot;
import org.firstinspires.ftc.teamcode.robot.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.util.math.MathStuff;

import java.util.ArrayList;
import java.util.List;
/**
 * Independent relay-feedback test OpMode.
 * Estimates PID coefficients through relay oscillations
 * and prescribed phase margins.
 * **Made with passion , used with love for precision.**
 */
@Autonomous(name = "Relay PID AutoTune", group = "Tests")
public class RelayFeedBackAutoTune extends LinearOpMode {


    public Robot robot;
    Localizer localizer;
    private static final double FORWARD_POWER = 0.15;
    private static final double STRAFE_POWER = 0.15;
    private static final double HEADING_POWER = 0.12;
    private static final double POSITION_HYSTERESIS = 0.15;
    private static final double MAX_POSITION_EXCURSION = 8.0;
    private static final double MAX_UNTESTED_TRANSLATION_DRIFT = 2.0;
    private static final double HEADING_HYSTERESIS = Math.toRadians(1);
    private static final double MAX_HEADING_EXCURSION = Math.toRadians(35);
    private static final int FORWARD_COMMAND_SIGN = 1;
    private static final int STRAFE_COMMAND_SIGN = 1;
    private static final int HEADING_COMMAND_SIGN = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, Alliance.RED).setOpModeType(Robot.OpModeType.AUTO).initialize();
        localizer = new Localizer();
        Config forwardConfig = Config.translation(FORWARD_POWER,
                POSITION_HYSTERESIS, MAX_POSITION_EXCURSION,
                MAX_UNTESTED_TRANSLATION_DRIFT);
        Config strafeConfig = Config.translation(STRAFE_POWER,
                POSITION_HYSTERESIS, MAX_POSITION_EXCURSION,
                MAX_UNTESTED_TRANSLATION_DRIFT);
        Config headingConfig = Config.heading(HEADING_POWER,
                HEADING_HYSTERESIS, MAX_HEADING_EXCURSION,
                MAX_UNTESTED_TRANSLATION_DRIFT);
        forwardConfig.commandSign = FORWARD_COMMAND_SIGN;
        strafeConfig.commandSign = STRAFE_COMMAND_SIGN;
        headingConfig.commandSign = HEADING_COMMAND_SIGN;

        telemetry.addLine("Relay PID AutoTune: FORWARD -> STRAFE -> HEADING");
        telemetry.addLine("No existing PID gains required; PathFollower is not called.");
        telemetry.addLine("Check units/signs. Place robot stationary before START.");
        telemetry.addLine("CROSS continues between tests. CIRCLE cancels.");
        telemetry.update();
        waitForStart();
        if (!opModeIsActive()) return;

        Result forwardResult = null;
        Result strafeResult = null;
        Result headingResult = null;
        String status;
        try {
            forwardResult = runAxis(this, localizer, Axis.FORWARD, forwardConfig);
            forwardResult.display(this, Axis.FORWARD);
            if (waitForNext("Forward complete. Wait for rest; A begins strafe.")) return;
            strafeResult = runAxis(this, localizer, Axis.STRAFE, strafeConfig);
            strafeResult.display(this, Axis.STRAFE);
            if (waitForNext("Strafe complete. Wait for rest; A begins heading.")) return;
            headingResult = runAxis(this, localizer, Axis.HEADING, headingConfig);
            status = "Finished. Candidate gains only; no gains applied or saved to Constants.";
        } catch (IllegalStateException | IllegalArgumentException failure) {
            status = "Test stopped: " + failure.getMessage();
        } finally {
            Drivetrain.betterDrive(0, 0, 0);
        }

        while (opModeIsActive()) {
            telemetry.addLine(status);
            if (forwardResult != null) forwardResult.display(this, Axis.FORWARD);
            if (strafeResult != null) strafeResult.display(this, Axis.STRAFE);
            if (headingResult != null) headingResult.display(this, Axis.HEADING);
            telemetry.addLine("Validate PID conventions and trajectory tracking before applying.");
            telemetry.update();
            idle();
        }
    }

    private boolean waitForNext(String message) throws InterruptedException {
        telemetry.addLine(message);
        telemetry.addLine("No automatic return; each test captures its actual starting pose.");
        telemetry.update();
        while (opModeIsActive() && gamepad1.crossWasPressed() && !gamepad1.circleWasPressed()) idle();
        while (opModeIsActive() && !gamepad1.crossWasPressed() && !gamepad1.circleWasPressed()) idle();
        if (!opModeIsActive() || gamepad1.circleWasPressed()) return true;
        while (opModeIsActive() && gamepad1.crossWasPressed() && !gamepad1.circleWasPressed()) idle();
        return !opModeIsActive() || gamepad1.circleWasPressed();
    }

    public enum Axis { FORWARD, STRAFE, HEADING }

    public static final class Config {
        public double commandAmplitude;
        public double hysteresis;
        public double maxAxisDisplacement;
        public double maxTranslationDrift;
        public double maxHeadingDrift = Math.toRadians(2);
        public double timeoutSeconds = 5;
        public double maxSampleGapSeconds = 0.1;
        public double maxRelativeSpread = 0.20;
        public int warmupCycles = 2;
        public int measuredCycles = 5;
        public int minSamplesPerCycle = 20;
        public double phaseMarginRadians = Math.toRadians(45);
        public double derivativeIntegralRatio = 0.25;
        public int commandSign = 1;

        public static Config translation(double power, double epsilon,
                                         double axisLimit, double driftLimit) {
            Config c = new Config();
            c.commandAmplitude = power;
            c.hysteresis = epsilon;
            c.maxAxisDisplacement = axisLimit;
            c.maxTranslationDrift = driftLimit;
            return c;
        }
        public static Config heading(double power, double epsilonRadians,
                                     double angleLimitRadians, double driftLimit) {
            return translation(power, epsilonRadians, angleLimitRadians, driftLimit);
        }
        private void validate() {
            if (!(commandAmplitude > 0 && commandAmplitude <= 1)
                    || !(hysteresis > 0 && maxAxisDisplacement > hysteresis)
                    || !(maxTranslationDrift > 0 && maxHeadingDrift > 0)
                    || !(timeoutSeconds > 0 && maxSampleGapSeconds > 0)
                    || !(maxRelativeSpread > 0)
                    || warmupCycles < 0 || measuredCycles < 3
                    || minSamplesPerCycle < 4
                    || !(phaseMarginRadians > 0 && phaseMarginRadians < Math.PI / 2)
                    || !(derivativeIntegralRatio > 0)
                    || (commandSign != 1 && commandSign != -1)) {
                throw new IllegalArgumentException("Invalid relay configuration");
            }
        }
    }

    public static final class Result {
        public final double amplitude, period, processMagnitude, phaseOffset;
        public final double kp, ki, kd;
        public final double amplitudeSpread, periodSpread;
        Result(double amplitude, double period, double processMagnitude,
               double phaseOffset, double kp, double ki, double kd,
               double amplitudeSpread, double periodSpread) {
            this.amplitude = amplitude; this.period = period;
            this.processMagnitude = processMagnitude; this.phaseOffset = phaseOffset;
            this.kp = kp; this.ki = ki; this.kd = kd;
            this.amplitudeSpread = amplitudeSpread; this.periodSpread = periodSpread;
        }
        public void display(LinearOpMode op, Axis axis) {
            op.telemetry.addData(axis + " candidate PID", "P=%.8f I=%.8f D=%.8f", kp, ki, kd);
            op.telemetry.addData(axis + " oscillation", "amplitude=%.5f period=%.5fs", amplitude, period);
            op.telemetry.addData(axis + " spread", "amplitude=%.1f%% period=%.1f%%",
                    amplitudeSpread * 100, periodSpread * 100);
        }
    }


    public static Result runAxis(LinearOpMode op, Localizer loc,
                                 Axis axis, Config c) throws InterruptedException {
        c.validate();
        try {
            if (!op.opModeIsActive()) throw new IllegalStateException("OpMode not active");
            loc.update();
            double x0 = loc.getCurrentPose().getX();
            double y0 = loc.getCurrentPose().getY();
            double h0 = loc.getHeading(AngleUnit.RADIANS);
            requireFinite(x0, y0, h0);
            double cos = Math.cos(h0), sin = Math.sin(h0);
            double started = seconds(), previousTime = started;
            double cycleStart = Double.NaN;
            double lo = 0, hi = 0;
            int state = 1, cycleSamples = 0, completed = 0;
            List<Double> periods = new ArrayList<>();
            List<Double> amplitudes = new ArrayList<>();

            while (op.opModeIsActive()) {
                if (op.gamepad1.b) throw new IllegalStateException("Cancelled with B");
                loc.update();
                double now = seconds();
                if (now - started > c.timeoutSeconds) throw new IllegalStateException("Relay timeout");
                if (now - previousTime > c.maxSampleGapSeconds) throw new IllegalStateException("Sampling gap too large");
                previousTime = now;
                double dx = loc.getCurrentPose().getX() - x0;
                double dy = loc.getCurrentPose().getY() - y0;
                double dh = MathStuff.normalizeRadians(loc.getHeading(AngleUnit.RADIANS) - h0);
                requireFinite(dx, dy, dh);
                double forward = cos * dx + sin * dy;
                double lateral = -sin * dx + cos * dy;
                double measured = axis == Axis.FORWARD ? forward
                        : axis == Axis.STRAFE ? lateral : dh;
                double drift = axis == Axis.FORWARD ? Math.abs(lateral)
                        : axis == Axis.STRAFE ? Math.abs(forward) : Math.hypot(dx, dy);
                if (Math.abs(measured) > c.maxAxisDisplacement
                        || drift > c.maxTranslationDrift
                        || (axis != Axis.HEADING && Math.abs(dh) > c.maxHeadingDrift)) {
                    throw new IllegalStateException("Motion outside relay test limits");
                }

                if (!Double.isNaN(cycleStart)) {
                    lo = Math.min(lo, measured); hi = Math.max(hi, measured);
                    cycleSamples++;
                }
                double error = -measured;
                int next = state;
                if (error > c.hysteresis) next = 1;
                else if (error < -c.hysteresis) next = -1;

                if (state == 1 && next == -1) {
                    if (!Double.isNaN(cycleStart)) {
                        completed++;
                        if (completed > c.warmupCycles) {
                            if (cycleSamples < c.minSamplesPerCycle)
                                throw new IllegalStateException("Too few samples in a cycle");
                            periods.add(now - cycleStart);
                            amplitudes.add((hi - lo) / 2.0);
                        }
                        if (periods.size() >= c.measuredCycles) return calculate(c, amplitudes, periods);
                    }
                    cycleStart = now; lo = hi = measured; cycleSamples = 0;
                }
                state = next;
                double command = c.commandSign * state * c.commandAmplitude;
                Drivetrain.betterDrive(axis == Axis.FORWARD ? command : 0,
                        axis == Axis.STRAFE ? command : 0,
                        axis == Axis.HEADING ? command : 0);
                op.telemetry.addData("Relay axis", axis);
                op.telemetry.addData("Displacement / error", "%.5f / %.5f", measured, error);
                op.telemetry.addData("Relay command", command);
                op.telemetry.addData("Measured cycles", "%d / %d", periods.size(), c.measuredCycles);
                op.telemetry.addLine("B cancels. Untested axes have zero command.");
                op.telemetry.update();
                op.idle();
            }
            throw new IllegalStateException("OpMode stopped before measurement completed");
        } finally {
            Drivetrain.betterDrive(0, 0, 0);
        }
    }

    private static Result calculate(Config c, List<Double> amplitudes, List<Double> periods) {
        double sigma = average(amplitudes), period = average(periods);
        double aSpread = relativeSpread(amplitudes, sigma);
        double tSpread = relativeSpread(periods, period);
        if (aSpread > c.maxRelativeSpread || tSpread > c.maxRelativeSpread)
            throw new IllegalStateException("Oscillation is not repeatable; no gains returned");
        if (!(sigma > 1.5 * c.hysteresis && period > 0))
            throw new IllegalStateException("Oscillation too close to hysteresis thresholds");
        double rp = Math.PI * sigma / (4.0 * c.commandAmplitude);
        double phase = Math.asin(c.hysteresis / sigma);
        double desiredPhase = c.phaseMarginRadians - phase;
        if (!(Math.abs(desiredPhase) < Math.PI / 2))
            throw new IllegalStateException("Requested phase outside ideal PID phase range");
        double b = Math.tan(desiredPhase);
        double mu = c.derivativeIntegralRatio;
        double q = (b + Math.sqrt(b * b + 4 * mu)) / (2 * mu);
        double ti = q / (2 * Math.PI / period);
        double td = mu * ti;
        double kp = 1 / (rp * Math.sqrt(1 + b * b));
        double ki = kp / ti, kd = kp * td;
        requireFinite(kp, ki, kd);
        return new Result(sigma, period, rp, phase, kp, ki, kd, aSpread, tSpread);
    }
    private static double average(List<Double> values) {
        double sum = 0; for (double x : values) sum += x;
        return sum / values.size();
    }
    private static double relativeSpread(List<Double> values, double mean) {
        double min = Double.POSITIVE_INFINITY, max = Double.NEGATIVE_INFINITY;
        for (double x : values) { min = Math.min(min, x); max = Math.max(max, x); }
        return (max - min) / mean;
    }
    private static double seconds() { return System.nanoTime() * 1e-9; }
    private static void requireFinite(double... values) {
        for (double x : values) if (Double.isNaN(x) || Double.isInfinite(x))
            throw new IllegalStateException("Non-finite localizer measurement or gain");
    }
}
