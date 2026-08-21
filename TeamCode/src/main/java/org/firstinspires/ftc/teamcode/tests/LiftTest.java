package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.globals.Alliance;
import org.firstinspires.ftc.teamcode.robot.globals.Robot;
import org.firstinspires.ftc.teamcode.robot.subsystem.Hang;

@Disabled
public class LiftTest extends LinearOpMode {
    Robot r;
    public void runOpMode() throws InterruptedException {
        r = new Robot(hardwareMap, telemetry, Alliance.RED)
                .setOpModeType(Robot.OpModeType.TELEOP)
                .initialize();

        waitForStart();
        while ( opModeIsActive() ) {
            Hang.hangServosLeft[0].setPower(0.1 * (gamepad1.right_trigger - gamepad1.left_trigger));
            Hang.hangServosRight[0].setPower(0.1 * (gamepad1.right_trigger - gamepad1.left_trigger));

            telemetry.addData("power LEFT", Hang.hangServosLeft[0].getPower());
            telemetry.addData("power RIGHT", Hang.hangServosRight[0].getPower());
            telemetry.update();
            r.update();
        }

    }
}
