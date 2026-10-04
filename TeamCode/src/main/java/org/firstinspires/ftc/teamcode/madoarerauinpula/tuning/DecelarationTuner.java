package org.firstinspires.ftc.teamcode.madoarerauinpula.tuning;

import static org.firstinspires.ftc.teamcode.robot.globals.Alliance.RED;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.madoarerauinpula.Localizer;
import org.firstinspires.ftc.teamcode.madoarerauinpula.PathPlanning.Pose2D;
import org.firstinspires.ftc.teamcode.robot.globals.Robot;
import org.firstinspires.ftc.teamcode.robot.subsystem.Drivetrain;

import java.util.ArrayList;
import java.util.List;

@TeleOp
public class DecelarationTuner extends LinearOpMode {

    public boolean stopped  , running , finished , forward;
    public double decelvelocity = 2 ;
    public double accellpower = 1;

    public double targetvelocity = 30;
    public double result = 30;

    public Robot robot;

    public List<Double> accelerations = new ArrayList<>();
    public double previousvelo = 0;
    public long previoustime ;

    public Localizer localizer;


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, RED).setOpModeType(Robot.OpModeType.TELEOP);
        robot.initialize();
        localizer = new Localizer();
        localizer.setStartingPose(new Pose2D(72, 72 , Math.toRadians(180)));

        telemetry.addLine("Zero Power Deceleration Tuner");
        telemetry.addLine("X = start FORWARD (X)");
        telemetry.addLine("CIRCLE = start STRAFE (Y)");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()){
            localizer.update();

            if(gamepad1.crossWasPressed() && !running) startTest(true);
            if(gamepad1.circleWasPressed() && !running) startTest(false);

            if(running && !finished) runTest();

            telemetry.addData("Running", running);
            telemetry.addData("Coasting", stopped);
            telemetry.addData("Velocity", getChosenVelocity());
            telemetry.addData("Samples", accelerations.size());

            if (finished) {
                telemetry.addLine("---------------------------");
                telemetry.addData("Deceleration (in/s²)", result);
                telemetry.addLine("Pune în Localizer:");
                if (forward) {
                    telemetry.addData("xDeceleration", result);
                } else {
                    telemetry.addData("yDeceleration", result);
                }
                telemetry.addLine("Apasă A/B din nou pentru alt test");
            }

            telemetry.update();
            robot.update();
        }

    }

    public void runTest() {
        double currentVel = getChosenVelocity();

        if (!stopped) {
            if (Math.abs(currentVel) > targetvelocity) {
                previousvelo = currentVel;
                previoustime = System.nanoTime();
                stopped = true;
                Drivetrain.betterDrive(0, 0 ,  0);
            }
        } else {
            long now = System.nanoTime();
            double dt = (now - previoustime) / 1e9;

            if (dt > 1e-6) {
                double a = (currentVel - previousvelo) / dt;
                accelerations.add(a);
            }

            previousvelo = currentVel;
            previoustime = now;

            if (Math.abs(currentVel) < decelvelocity) {
                finishTest();
            }
        }
    }
    private void finishTest() {
        Drivetrain.betterDrive(0 ,0 , 0);
        running = false;
        finished = true;

        if (accelerations.isEmpty()) {
            result = 0;
            return;
        }

        double sum = 0;
        for (double a : accelerations) {
            sum += a;
        }
        double average = sum / accelerations.size();

        result = Math.abs(average);
    }

    private double getChosenVelocity() {
        double heading = localizer.getHeading(AngleUnit.RADIANS);
        double xVel = localizer.getXVelocity();
        double yVel = localizer.getYVelocity();

        double xRobot = xVel * Math.cos(-heading) - yVel * Math.sin(-heading);
        double yRobot = xVel * Math.sin(-heading) + yVel * Math.cos(-heading);

        return forward ? xRobot : yRobot;
    }

    private void startTest(boolean forward) {
        this.forward = forward;
        accelerations.clear();
        running = true;
        stopped = false;
        finished = false;
        result = 0;

       if (forward) {
            Drivetrain.betterDrive(accellpower, 0, 0);
        } else {
            Drivetrain.betterDrive(0, accellpower, 0);
        }
    }
}
