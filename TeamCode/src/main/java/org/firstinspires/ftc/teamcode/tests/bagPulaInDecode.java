package org.firstinspires.ftc.teamcode.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.globals.Alliance;
import org.firstinspires.ftc.teamcode.robot.globals.Robot;
import org.firstinspires.ftc.teamcode.robot.subsystem.Blocker;
import org.firstinspires.ftc.teamcode.robot.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystem.Storage;
import org.firstinspires.ftc.teamcode.tasks.seasonal_tasks.Shoot;
import org.firstinspires.ftc.teamcode.util.BetterOpMode;
import org.firstinspires.ftc.teamcode.util.MultipleTelemetry;
import org.firstinspires.ftc.teamcode.util.wrappers.BetterGamepad;

@TeleOp
@Configurable
public class bagPulaInDecode extends BetterOpMode {

    Robot robot;
    MultipleTelemetry telemetry;
    public static double targetVelocity;
    @Override
    public void initialize() {
        robot = new Robot(hardwareMap, telemetry, Alliance.BLUE)
                .setOpModeType(Robot.OpModeType.TELEOP)
                .setPlayStyle(Robot.PlayStyle.CLOSE)
                .initialize();
        telemetry = new MultipleTelemetry(super.telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        gamepadEx1.getButton(BetterGamepad.Buttons.LEFT_BUMPER)
                .whenPressed(Intake::toggle, BetterGamepad.Type.MAIN);
        gamepadEx1.getButton(BetterGamepad.Buttons.RIGHT_BUMPER)
                .whenPressed(Shoot::new, BetterGamepad.Type.MAIN);
        gamepadEx1.getButton(BetterGamepad.Buttons.OPTIONS)
                .whenPressed(robot::resetPinPoint, BetterGamepad.Type.MAIN);
        gamepadEx1.getButton(BetterGamepad.Buttons.CROSS)
                .whenPressed(() -> Shooter.shooterEnabled = !Shooter.shooterEnabled, BetterGamepad.Type.PARALLEL);
        gamepadEx1.getButton(BetterGamepad.Buttons.SHARE)
                .whenPressed(Robot::switchPlayStyle, BetterGamepad.Type.MAIN);

        Blocker.block();

    }

    @Override
    public void initialize_loop() {

    }

    @Override
    public void on_start() {

    }

    @Override
    public void active_loop() {
        robot.update();

        telemetry.addData("target velocity", targetVelocity);
        telemetry.addData("current velocity", Math.abs(Shooter.motor2.getVelocity()));

        Shooter.tuningupdate(targetVelocity);
        Intake.update();
        Storage.update2();

        telemetry.update();
    }

    @Override
    public void end() {

    }
}
