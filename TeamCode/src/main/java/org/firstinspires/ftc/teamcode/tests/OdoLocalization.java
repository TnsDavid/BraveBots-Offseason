package org.firstinspires.ftc.teamcode.tests;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.robot.globals.Alliance;


@Disabled
public class OdoLocalization extends LinearOpMode {

    GoBildaPinpointDriver odo;
    Alliance a;
    Pose robotPose ;
    public double dtwidth = 0.432, dtlength = 0.435;

    public void setOdo() {
        odo.setPosX(a== Alliance.RED ? dtwidth / 2 : 3.65-(dtwidth/2), DistanceUnit.METER);
        odo.setPosY(dtlength / 2, DistanceUnit.METER);
        robotPose=new Pose(odo.getPosX(DistanceUnit.METER),odo.getPosY(DistanceUnit.METER),odo.getHeading(AngleUnit.RADIANS));
    }

    @Override
    public void runOpMode() throws InterruptedException {
        a = Alliance.BLUE;
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "nigg");
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.setEncoderResolution(34.310551 , DistanceUnit.MM);
        odo.setOffsets(Constants.xOffset,Constants.yOffset, DistanceUnit.INCH);
        waitForStart();
        setOdo();
        while (opModeIsActive()) {
            odo.update();
            telemetry.addData("x",robotPose.getX());
            telemetry.addData("y", robotPose.getY());
            telemetry.addData("heading", odo.getHeading(AngleUnit.DEGREES));
            telemetry.update();


        }
    }
}
