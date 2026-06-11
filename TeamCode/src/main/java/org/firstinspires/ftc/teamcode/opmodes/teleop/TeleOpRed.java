package org.firstinspires.ftc.teamcode.opmodes.teleop;

import org.firstinspires.ftc.teamcode.robot.Alliance;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp Red", group = "TeleOp")
public class TeleOpRed extends TeleOp {

    @Override
    public void setAlliance() {
        super.a = Alliance.RED;
    }
}
