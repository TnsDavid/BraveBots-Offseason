package org.firstinspires.ftc.teamcode.tests;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.globals.Alliance;
import org.firstinspires.ftc.teamcode.robot.globals.Robot;
import org.firstinspires.ftc.teamcode.robot.subsystem.Blocker;
import org.firstinspires.ftc.teamcode.robot.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystem.Storage;
import org.firstinspires.ftc.teamcode.robot.subsystem.Turret;
import org.firstinspires.ftc.teamcode.tasks.seasonal_tasks.Shoot;
import org.firstinspires.ftc.teamcode.util.BetterOpMode;
import org.firstinspires.ftc.teamcode.util.MultipleTelemetry;
import org.firstinspires.ftc.teamcode.util.math.ShooterMath;
import org.firstinspires.ftc.teamcode.util.wrappers.BetterGamepad;

import java.util.function.ToDoubleFunction;

@TeleOp
@Configurable
public class bagPulaInDecode extends BetterOpMode {

    Robot robot;
    MultipleTelemetry telemetry;
    public static double targetVelocity, targetHoodPos;
    @Override
    public void initialize() {
        robot = new Robot(hardwareMap, telemetry, Alliance.RED)
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
        gamepadEx2.getButton(BetterGamepad.Buttons.CROSS)
                .whenPressed(() -> Shooter.shooterEnabled = !Shooter.shooterEnabled, BetterGamepad.Type.PARALLEL);
        gamepadEx1.getButton(BetterGamepad.Buttons.SHARE)
                .whenPressed(Robot::switchPlayStyle, BetterGamepad.Type.MAIN);
        gamepadEx2.getButton(BetterGamepad.Buttons.TOUCHPAD)
                        .whenPressed(Drivetrain::toggleControlMethod, BetterGamepad.Type.PARALLEL);
        gamepadEx2.getButton(BetterGamepad.Buttons.SQUARE)
                        .whenPressed(() -> Shooter.offset += 10, BetterGamepad.Type.PARALLEL);
        gamepadEx2.getButton(BetterGamepad.Buttons.CIRCLE)
                        .whenPressed(() -> Shooter.offset -= 10, BetterGamepad.Type.PARALLEL);

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
        targetVelocity = (Robot.playStyle == Robot.PlayStyle.CLOSE) ?
                ShooterMath.calculateVelocityFromDistance(Turret.virtualDist) :
                ShooterMath.calculateVelocityFARFromDistance(Turret.virtualDist);
        targetVelocity += Shooter.offset;
        targetHoodPos = Shooter.readyForShoot ?
                ShooterMath.calculateHoodAngleFromVelocity(Math.abs(Shooter.motor2.getVelocity())) :
                ShooterMath.calculateHoodAngleFromDistance(Turret.virtualDist);
        //double hoodPos = Shooter.hoodServo.getPosition() +
          //      (0.008 * (gamepadEx1.getDouble(BetterGamepad.Trigger.RIGHT_TRIGGER) - gamepadEx1.getDouble(BetterGamepad.Trigger.LEFT_TRIGGER)));
        Shooter.tuningupdate(targetVelocity, targetHoodPos);
        Intake.update();
        Storage.update2();
        Turret.update();
        Drivetrain.betterDrive(gamepadEx1.getDouble(BetterGamepad.Trigger.LEFT_X), -gamepadEx1.getDouble(BetterGamepad.Trigger.LEFT_Y), gamepadEx1.getDouble(BetterGamepad.Trigger.RIGHT_X));

        telemetry.update();
    }

    @Override
    public void end() {

    }
}
