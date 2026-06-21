package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.Robot;
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
        Turret.setNeutralPosition(0.5);
        gamepadEx1.getButton(BetterGamepad.Buttons.CROSS)
                .whenPressed(Intake::toggle, BetterGamepad.Type.MAIN);
        gamepadEx1.getButton(BetterGamepad.Buttons.RIGHT_BUMPER)
                .whenPressed(Shoot::new, BetterGamepad.Type.MAIN);
    }
    @Override
    public void initialize_loop() {

    }
    @Override
    public void on_start() {

    }
    @Override
    public void active_loop() {
        Turret.setNeutralPosition(Turret.getPositon() + 0.01 * gamepadEx1.getDouble(BetterGamepad.Trigger.RIGHT_X));
        Shooter.motor1.update(Shooter.velocity);
        Shooter.motor2.update(Shooter.velocity);
        Shooter.servo.setPosition(Shooter.servo.getPosition() + 0.02 * (gamepadEx1.getDouble(BetterGamepad.Trigger.RIGHT_TRIGGER) - gamepadEx1.getDouble(BetterGamepad.Trigger.LEFT_TRIGGER)));

        Intake.update();
        Storage.update();
        telemetry.addData("turret", Turret.getPositon());
        telemetry.addData("hood", Shooter.servo.getPosition());
        telemetry.update();
        r.update();

    }
    @Override
    public void end() {
    }
}
