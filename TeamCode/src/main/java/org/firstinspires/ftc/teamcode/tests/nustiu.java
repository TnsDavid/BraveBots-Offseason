package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.globals.Alliance;
import org.firstinspires.ftc.teamcode.robot.globals.Robot;
import org.firstinspires.ftc.teamcode.robot.subsystem.Shooter;

@TeleOp(name = "MUIE LUI VASILE", group = "Vasile este Prost")
public class nustiu extends LinearOpMode {
    Robot r;

    public void runOpMode() throws InterruptedException {
        r = new Robot(hardwareMap, telemetry, Alliance.RED)
                .setOpModeType(Robot.OpModeType.TELEOP)
                .initialize();
        waitForStart();
        while( opModeIsActive() ) {
            if( gamepad1.crossWasPressed() ) {
                Shooter.motor1.setPower(1);
                Shooter.motor2.setPower(1);
            }
            if( gamepad1.triangleWasPressed() ) {
                Shooter.motor1.setPower(0);
                Shooter.motor2.setPower(0);
            }
            telemetry.addData("m1 vel", Shooter.motor2.getVelocity());
            telemetry.addData("m2 vel", Shooter.motor2.getVelocity());
            telemetry.update();
            r.update();
        }

    }

}
