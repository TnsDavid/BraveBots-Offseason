package org.firstinspires.ftc.teamcode.opmodes.teleOp;

import org.firstinspires.ftc.teamcode.robot.globals.Alliance;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp Blue", group = "TeleOp")
public class TeleOpBlue extends TeleOp {

    @Override
    public void setAlliance() {
        super.a = Alliance.BLUE;
    }
}
