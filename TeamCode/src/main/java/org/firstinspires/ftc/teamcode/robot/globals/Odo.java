package org.firstinspires.ftc.teamcode.robot.globals;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.madoarerauinpula.Constants;

public class Odo {
    public static GoBildaPinpointDriver odo;

    public static void init(HardwareMap hm, Robot.OpModeType opModeType, Robot.PlayStyle playStyle){
        if(odo == null){
            odo = hm.get(GoBildaPinpointDriver.class, "pinpoint");
            odo.setOffsets(Constants.yOffset, Constants.xOffset, DistanceUnit.INCH);
            odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
            odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            odo.resetPosAndIMU();
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
    }
}
