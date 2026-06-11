package org.firstinspires.ftc.teamcode.opmodes.teleop;

import org.firstinspires.ftc.teamcode.robot.Alliance;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp Blue", group = "TeleOp")
public class TeleOpBlue extends TeleOp {

    @Override
    public void setAlliance() {
        super.a = Alliance.BLUE;
    }
}
