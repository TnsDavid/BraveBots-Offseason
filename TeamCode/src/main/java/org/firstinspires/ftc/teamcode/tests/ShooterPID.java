package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorController;

import org.firstinspires.ftc.teamcode.robot.globals.Alliance;
import org.firstinspires.ftc.teamcode.robot.globals.Robot;
import org.firstinspires.ftc.teamcode.robot.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.tasks.command_based.core.Scheduler;
import org.firstinspires.ftc.teamcode.tasks.seasonal_tasks.Shoot;
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
        waitForStart();
        while( opModeIsActive() ) {
            robot.update();
            Shooter.tuningupdate(velocity);
            Shooter.hoodServo.setPosition(Shooter.hoodServo.getPosition() +
                    0.025 * (gamepad1.left_trigger - gamepad1.right_trigger)
            );
            if( gamepad1.crossWasPressed() )
                Intake.toggle();
            if( gamepad1.rightBumperWasPressed() )
                Scheduler.now(new Shoot());
            telemetry.addData("hoodpos", Shooter.hoodServo.getPosition());
            telemetry.addData(" target velocity", Shooter.velocity);
            telemetry.addData("curr vel", Shooter.motor2.getVelocity());
            telemetry.update();
            robot.update();
            Intake.update();
            Scheduler.update();
        }
    }

    // s=0.15 p=0.6 f=0.92
}
