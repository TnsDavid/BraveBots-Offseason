package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.robot.Robot.alliance;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystem.Turret;

@TeleOp(name = "Localizer Test", group = "Teste")
public class LocalizerTest extends LinearOpMode {
    Robot r;
    Follower f;

    public double dtlength=.356;
    public double dtwidth=.38;
    @Override
    public void runOpMode() throws InterruptedException{
        r = new Robot(hardwareMap, telemetry, Alliance.RED).setOpModeType(Robot.OpModeType.TELEOP).initialize();
        Turret.turret1.setPosition(0.5);
        Turret.turret2.setPosition(0.5);
        Shooter.hoodServo.setPosition(Shooter.minHoodpos);
        f= Constants.createFollower(hardwareMap);
        Pose resetPose = new Pose ((alliance == Alliance.RED ? dtwidth / 2 * 39.37 : (3.65 - (dtwidth / 2)) * 39.37) , (dtlength / 2 ) * 39.37 , Math.toRadians(90));
        waitForStart();
        while(opModeIsActive()){
            r.update();
            f.update();
            if(gamepad1.optionsWasPressed()){
                r.resetPinPoint();
            }

            if(gamepad1.crossWasPressed()){
                f.setPose(resetPose);
            }
            Drivetrain.drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            telemetry.addData("x", Robot.robotPose.getX());
            telemetry.addData("y", Robot.robotPose.getY());
            telemetry.addData("heading", Robot.robotPose.getHeading());

            telemetry.addLine("\n--------------\n");

            telemetry.addData("fx" , f.getPose().getX() / 39.37);
            telemetry.addData("fy" , f.getPose().getY() / 39.37);
            telemetry.addData("fheading" , f.getPose().getHeading());
            telemetry.update();
        }
    }
}
