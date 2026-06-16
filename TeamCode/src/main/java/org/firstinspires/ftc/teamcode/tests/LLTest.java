package org.firstinspires.ftc.teamcode.tests;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.util.math.LLMath;

import java.util.LinkedList;
import java.util.List;

@TeleOp(name = "Test LimeLight", group = "Teste")
public class LLTest extends LinearOpMode {
    Robot r;
    Pose p;
    LinkedList<Pose> unTereaWindFuseVaRog;
    @Override
    public void runOpMode() throws InterruptedException{
        r = new Robot(hardwareMap, telemetry, Alliance.RED)
                .setOpModeType(Robot.OpModeType.TELEOP)
                .initialize();
        p = new Pose();
        //setez cv in odometrie plm
        waitForStart();
        while(opModeIsActive()){
            if( gamepad1.crossWasPressed()) {
                LLMath.update();
                unTereaWindFuseVaRog = LLMath.getRobotRelativePoses();
            }
            if( gamepad1.optionsWasPressed() )
                r.resetPinPoint();
            if( gamepad1.shareWasPressed() )
                Robot.switchPlayStyle();
            if( unTereaWindFuseVaRog != null ) {
                for (Pose results : unTereaWindFuseVaRog) {
                    Pose fieldRel = LLMath.getFieldRelativePose(results);
                    telemetry.addData(" x", results.getX());
                    telemetry.addData(" y", results.getY());
                    telemetry.addData("field x", fieldRel.getX());
                    telemetry.addData("field y", fieldRel.getY());
                    telemetry.addLine("----------------------");
                }
            }
            telemetry.addLine("TEURI");
            if(LLMath.results!=null)
            {
                for(Pose teuri: LLMath.results)
                {
                    telemetry.addData("tx", teuri.getX());
                    telemetry.addData("ty", teuri.getY());
                    telemetry.addLine("----------------------");
                }
            }
            Drivetrain.betterDrive(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
            telemetry.addData("playstyle", Robot.playStyle);
            telemetry.addData("robot x", Robot.robotPose.getX());
            telemetry.addData("robot y", Robot.robotPose.getY());
            telemetry.update();
            r.update();
        }
    }
}
