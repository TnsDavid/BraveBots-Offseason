package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.subsystem.Intake;
import org.firstinspires.ftc.teamcode.util.BetterOpMode;
import org.firstinspires.ftc.teamcode.util.wrappers.BetterGamepad;
import org.firstinspires.ftc.teamcode.robot.subsystem.Drivetrain;

@TeleOp(name = "Field Centric Drivetrain", group = "Teste")
public class TestFCDrivetrain extends BetterOpMode {

    public MecanumDrive drive;
    public static boolean robotCentric = false;
    public double last, now, dt;
    public Robot r;

    public void initialize() {
        r = new Robot(hardwareMap, telemetry, Alliance.RED)
                .setOpModeType(Robot.OpModeType.TELEOP)
                .initialize();
        robotCentric = true;


        gamepadEx1.getButton(BetterGamepad.Buttons.OPTIONS)
                .whenPressed(() -> {
                    r.odo.setPosX(Constants.dtLength / 100, DistanceUnit.METER);
                    r.odo.setPosY(Constants.dtWidth / 100, DistanceUnit.METER);
                    r.odo.setHeading(Math.toRadians(90), AngleUnit.RADIANS);
                }, BetterGamepad.Type.MAIN);
        gamepadEx1.getButton(BetterGamepad.Buttons.CROSS)
                .whenPressed(Intake::toggle, BetterGamepad.Type.MAIN);

    }
    public void initialize_loop() {
        r.odo.update();
        r.update();
    }
    public void on_start() {
        r.odo.update();
        r.update();
        last = System.currentTimeMillis();
    }
    public void active_loop() {

        now = System.currentTimeMillis();
        dt = (now - last);
        last = now;
        telemetry.addData("hertz", 1000 / dt);

        if( gamepad1.touchpadWasPressed() )
            robotCentric = !robotCentric;
        Drivetrain.betterDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

        telemetry.addData("robot centric", robotCentric);
        telemetry.update();
        r.odo.update();
        r.update();
    }
    public void end() {

    }
}

