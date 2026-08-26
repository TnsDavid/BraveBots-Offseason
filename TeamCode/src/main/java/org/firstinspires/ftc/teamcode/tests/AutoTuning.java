package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.hardware.adafruit.AdafruitBNO055IMUNew;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robot.globals.Alliance;
import org.firstinspires.ftc.teamcode.robot.globals.Odo;
import org.firstinspires.ftc.teamcode.robot.globals.Robot;
import org.firstinspires.ftc.teamcode.robot.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystem.Storage;
import org.firstinspires.ftc.teamcode.robot.subsystem.Turret;
import org.firstinspires.ftc.teamcode.tasks.seasonal_tasks.Shoot;
import org.firstinspires.ftc.teamcode.util.BetterOpMode;
import org.firstinspires.ftc.teamcode.util.wrappers.BetterGamepad;

@TeleOp(name = "Auto Tuning", group = "Teste")
public class AutoTuning extends BetterOpMode {
    Robot r;
    @Override
    public void initialize() {
        r = new Robot(hardwareMap, telemetry, Alliance.RED).setOpModeType(Robot.OpModeType.TELEOP).initialize();
        //Turret.setNeutralPosition(0);
        gamepadEx1.getButton(BetterGamepad.Buttons.CROSS)
                .whenPressed(Intake::toggle, BetterGamepad.Type.MAIN);
        gamepadEx1.getButton(BetterGamepad.Buttons.RIGHT_BUMPER)
                .whenPressed(Shoot::new, BetterGamepad.Type.MAIN);
        gamepadEx1.getButton(BetterGamepad.Buttons.OPTIONS)
                .whenPressed(() -> {
                    r.resetPinPoint();
                    gamepad1.rumble(300);
                }, BetterGamepad.Type.MAIN);
        gamepadEx2.getButton(BetterGamepad.Buttons.SHARE)
                .whenPressed(Robot::switchPlayStyle, BetterGamepad.Type.MAIN);
        gamepadEx2.getButton(BetterGamepad.Buttons.RIGHT_BUMPER)
                .whenPressed(Turret::toggleStatus, BetterGamepad.Type.MAIN);
        gamepadEx2.getButton(BetterGamepad.Buttons.DPAD_RIGHT)
                .whenPressed(() -> Turret.offset -= .5, BetterGamepad.Type.PARALLEL);
        gamepadEx2.getButton(BetterGamepad.Buttons.DPAD_LEFT)
                .whenPressed(() -> Turret.offset += .5, BetterGamepad.Type.PARALLEL);
        gamepadEx2.getButton(BetterGamepad.Buttons.DPAD_UP)
                .whenPressed(() -> Turret.offset += 2, BetterGamepad.Type.PARALLEL);
        gamepadEx2.getButton(BetterGamepad.Buttons.DPAD_DOWN)
                .whenPressed(() -> Turret.offset -= 2, BetterGamepad.Type.PARALLEL);
    }
    @Override
    public void initialize_loop() {

    }
    @Override
    public void on_start() {

    }
    @Override
    public void active_loop() {
        //if( !Hang.raised) Hang.setPower(gamepadEx1.getDouble(BetterGamepad.Trigger.RIGHT_TRIGGER) - gamepadEx1.getDouble(BetterGamepad.Trigger.LEFT_TRIGGER));

        //Shooter.motor1.update(Shooter.velocity);
        //Shooter.motor2.update(Shooter.velocity);\
        double pos = Shooter.hoodServo.getPosition() + 0.02 * (gamepadEx1.getDouble(BetterGamepad.Trigger.RIGHT_TRIGGER) - gamepadEx1.getDouble(BetterGamepad.Trigger.LEFT_TRIGGER));
        Shooter.hoodServo.setPosition(pos);
        Shooter.tuningupdate(Shooter.velocity);
        Drivetrain.betterDrive(gamepadEx1.getDouble(BetterGamepad.Trigger.LEFT_X), -gamepadEx1.getDouble(BetterGamepad.Trigger.LEFT_Y), gamepadEx1.getDouble(BetterGamepad.Trigger.RIGHT_X) );
        Turret.update();
        Intake.update();
        Storage.update2();
        //Storage.update();
        telemetry.addData("turret", Turret.getPositon());
        telemetry.addData("hood", Shooter.hoodServo.getPosition());
        telemetry.addData("x", Odo.odo.getPosX(DistanceUnit.METER));
        telemetry.addData("y", Odo.odo.getPosY(DistanceUnit.METER));
        telemetry.addData("dist", Turret.virtualDist);
        telemetry.addData("heading", Odo.odo.getHeading(AngleUnit.RADIANS));
        telemetry.update();
        r.update();

    }
    @Override
    public void end() {
    }
}
