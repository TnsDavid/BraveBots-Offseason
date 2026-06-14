package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorController;

import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.util.wrappers.BetterMotorEx;

@TeleOp(name = "Shooter Tuning", group = "Teste")
@Configurable
public class ShooterPID extends LinearOpMode {
    public Robot robot;
    public static double velocity = 0;
    public BetterMotorEx m1, m2;
    public DcMotorController ch, eh;
    public static double p, i, d, f, s;

    @Override
    public void runOpMode() throws InterruptedException {
        ch = hardwareMap.get(DcMotorController.class, "Control Hub");
        eh = hardwareMap.get(DcMotorController.class, "Expansion Hub 2");
        robot= new Robot(hardwareMap, telemetry, Alliance.BLUE).setOpModeType(Robot.OpModeType.TELEOP).initialize();
        m1=robot.shooter;
        m2=robot.shooter2;
        m1.setPSFCoefficients(p, f, s);
        m2.setPSFCoefficients(p, f, s);
       // m1 = new BetterMotorEx(ch, 3);
        //m2 = new BetterMotorEx(eh, 3);
        waitForStart();
        while( opModeIsActive() ) {
            if( gamepad1.triangleWasPressed() ) {
                m1.setPSFCoefficients(p, f, s);
                m2.setPSFCoefficients(p, f, s);
                m1.setVelocity(velocity);
                m2.setVelocity(velocity);
            }
            if( gamepad1.circleWasPressed() ) {
                m1.setVelocity(0);
                m2.setVelocity(0);
            }

            telemetry.addData("m1 velo", m1.getVelocity());
            telemetry.addData("m2 velo", m2.getVelocity());
            telemetry.addData("p", p);
            telemetry.addData("f", f);
            telemetry.addData("target velo", velocity);
            telemetry.update();
            m1.update();
            //robot.update();
        }
    }
}
