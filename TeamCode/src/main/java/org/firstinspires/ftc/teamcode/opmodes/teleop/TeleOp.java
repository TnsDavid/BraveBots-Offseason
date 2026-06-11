package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.teamcode.robot.Alliance;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.subsystem.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystem.Shooter;
import org.firstinspires.ftc.teamcode.robot.subsystem.Storage;
import org.firstinspires.ftc.teamcode.robot.subsystem.Turret;

import org.firstinspires.ftc.teamcode.tasks.seasonal_tasks.ShootTest;
import org.firstinspires.ftc.teamcode.util.BetterOpMode;
import org.firstinspires.ftc.teamcode.util.MultipleTelemetry;
import org.firstinspires.ftc.teamcode.util.wrappers.BetterGamepad;

@Configurable
public abstract class TeleOp extends BetterOpMode {

    public Alliance a;
    boolean updateTurret = true;
    Robot robot;
    MultipleTelemetry telemetry;
    private boolean readyForRaise=false;
    public boolean shooterEnabled = true;
    volatile boolean shooterThreadRunning, turretThreadRunning;
    Thread shooterThread, turretThread;

    public abstract void setAlliance();

    @Override
    public void initialize() {
        setAlliance();
        telemetry = new MultipleTelemetry(super.telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        telemetry.setMsTransmissionInterval(300);
        robot = new Robot(hardwareMap, this.telemetry, this.a)
                .setOpModeType(Robot.OpModeType.TELEOP)
                .initialize();

        gamepadEx1.getButton(BetterGamepad.Buttons.CROSS)
                .whenPressed(Intake::toggle, BetterGamepad.Type.MAIN);
        gamepadEx1.getButton(BetterGamepad.Buttons.LEFT_BUMPER)
                .whenPressed(ShootTest::new, BetterGamepad.Type.MAIN);
        gamepadEx1.getButton(BetterGamepad.Buttons.OPTIONS)
                .whenPressed(() -> robot.resetPinPoint(), BetterGamepad.Type.MAIN);
        gamepadEx1.getButton(BetterGamepad.Buttons.SHARE)
                .whenPressed(Robot::switchPlayStyle, BetterGamepad.Type.MAIN);

        gamepadEx2.getButton(BetterGamepad.Buttons.CROSS)
                .whenPressed(() -> shooterEnabled = !shooterEnabled, BetterGamepad.Type.MAIN);
        gamepadEx2.getButton(BetterGamepad.Buttons.RIGHT_BUMPER)
                .whenPressed(() -> {
                    updateTurret = !updateTurret;
                    Turret.reseted = true;
                }, BetterGamepad.Type.MAIN);
        gamepadEx2.getButton(BetterGamepad.Buttons.OPTIONS)
                .whenPressed(() -> Drivetrain.robotCentric = !Drivetrain.robotCentric, BetterGamepad.Type.MAIN);
        gamepadEx2.getButton(BetterGamepad.Buttons.DPAD_RIGHT)
                .whenPressed(() -> Turret.offset -= .5, BetterGamepad.Type.MAIN);
        gamepadEx2.getButton(BetterGamepad.Buttons.DPAD_LEFT)
                .whenPressed(() -> Turret.offset += .5, BetterGamepad.Type.MAIN);

        shooterThread = new Thread(() -> {
            while( shooterThreadRunning && !Thread.currentThread().isInterrupted() ) {
                try {
                    Shooter.update();
                    Thread.sleep(3);
                } catch (InterruptedException e ) {
                    Thread.currentThread().interrupt();
                }
            }
        });
        turretThread = new Thread(() -> {
            while( turretThreadRunning && !Thread.currentThread().isInterrupted() ) {
                try {
                    if(updateTurret)
                        Turret.update();
                    else
                        Turret.setNeutralPosition(.5);
                    Thread.sleep(3);
                } catch( InterruptedException e ) {
                    Thread.currentThread().interrupt();
                }
            }
        });
    }

    @Override
    public void initialize_loop() {

    }

    @Override
    public void on_start() {
        turretThreadRunning = shooterThreadRunning = true;
        turretThread.start();
        shooterThread.start();
    }

    long now, last, dt;
    //volatile double slow;
    @Override
    public void active_loop() {
       // slow= gamepadEx1.getDouble(BetterGamepad.Trigger.LEFT_TRIGGER);
        now = System.nanoTime();
        dt = now - last;
        telemetry.addData("hz", 1e9 / dt);
        last = now;

        /*
        if( gamepadEx2.getButton(BetterGamepad.Buttons.TOUCHPAD).wasPressed() )
            readyForRaise = true;
        if( gamepadEx1.getButton(BetterGamepad.Buttons.TOUCHPAD).wasPressed() && readyForRaise )
            Scheduler.now(new Raise());
        */

        Drivetrain.betterDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
        Intake.update();
        Storage.update();
        robot.update();
        telemetry.update();
    }

    @Override
    public void end() {
        turretThreadRunning = shooterThreadRunning = false;
    }
}