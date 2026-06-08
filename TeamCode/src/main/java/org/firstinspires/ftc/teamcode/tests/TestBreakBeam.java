package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;

import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.util.wrappers.BreakBeam;

@TeleOp
@Configurable
public class TestBreakBeam extends LinearOpMode {
    BreakBeam bm , bmup;
    int portNumber = 3;
    DigitalChannelController controller;
    @Override
    public void runOpMode() throws InterruptedException{
        Robot r = new Robot(hardwareMap, telemetry, Alliance.RED).setOpModeType(Robot.OpModeType.TELEOP).initialize();
        controller = hardwareMap.get(DigitalChannelController.class,"Control Hub");
        bm = new BreakBeam(controller, portNumber);
        bmup = new BreakBeam(controller , portNumber);
        //bm.setMode(DigitalChannel.Mode.INPUT);

        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("status", bm.getBeamState());
            telemetry.addData("status up" , bmup.getBeamState());
            telemetry.update();
            r.update();
        }
    }
}
