package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;

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

    @Override
    public void runOpMode() throws InterruptedException {
        ch = hardwareMap.get(DcMotorController.class, "Control Hub");
        eh = hardwareMap.get(DcMotorController.class, "Expansion Hub 2");
        robot= new Robot(hardwareMap, telemetry, Alliance.BLUE).setOpModeType(Robot.OpModeType.TELEOP).initialize();
//        telemetry= new JoinedTelemetry(PanelsTelemetry.ftcTelemetry, telemetry);
        waitForStart();
        while( opModeIsActive() ) {
            robot.update();
            Shooter.tuningUpdate(velocity , 0);
            telemetry.addData(" target velocity", Shooter.velocity);
            telemetry.addData("curr vel", Shooter.motor1.getVelocity());
            telemetry.update();
        }
    }
}
