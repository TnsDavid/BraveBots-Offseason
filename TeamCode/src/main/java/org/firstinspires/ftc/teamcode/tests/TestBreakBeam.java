package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannelController;

import org.firstinspires.ftc.teamcode.robot.globals.Alliance;
import org.firstinspires.ftc.teamcode.robot.globals.Robot;
import org.firstinspires.ftc.teamcode.robot.subsystem.Storage;
import org.firstinspires.ftc.teamcode.util.wrappers.BreakBeam;

@TeleOp(name = "Test BreakBeam", group = "Teste")
@Configurable
public class TestBreakBeam extends LinearOpMode {
    BreakBeam bm , bmup;
    int portNumber = 0;
    DigitalChannelController controller;
    @Override
    public void runOpMode() throws InterruptedException{
        Robot r = new Robot(hardwareMap, telemetry, Alliance.RED).setOpModeType(Robot.OpModeType.TELEOP).initialize();
        //bm.setMode(DigitalChannel.Mode.INPUT);

        waitForStart();
        while(opModeIsActive()){
          //  telemetry.addData("status", bm.getBeamState());
            telemetry.addData("status up" , Storage.bUp.getBeamState());
            telemetry.addData("status middle", Storage.laserMiddle.getState());
            telemetry.addData("status down", Storage.bDown.getState());
            telemetry.update();
            r.update();
        }
    }
}
