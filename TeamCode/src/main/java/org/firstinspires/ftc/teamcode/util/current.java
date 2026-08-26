package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.robot.globals.Alliance;
import org.firstinspires.ftc.teamcode.robot.globals.Robot;
import org.firstinspires.ftc.teamcode.robot.subsystem.Drivetrain;

@TeleOp

public class current extends LinearOpMode {
    Robot r;

    public void runOpMode() throws InterruptedException {
        r = new Robot(hardwareMap, telemetry, Alliance.RED)
                .setOpModeType(Robot.OpModeType.TELEOP)
                        .initialize();
        telemetry.setMsTransmissionInterval(200);
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("fl power", Drivetrain.leftFront.getCurrent(AMPS));
            telemetry.addData("fr power", Drivetrain.rightFront.getCurrent(AMPS));
            telemetry.addData("bl power", Drivetrain.leftBack.getCurrent(AMPS));
            telemetry.addData("br power", Drivetrain.rightBack.getCurrent(AMPS));
            Drivetrain.betterDrive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
            telemetry.update();
        }
    }
}

