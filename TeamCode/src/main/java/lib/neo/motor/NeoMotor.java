package lib.neo.motor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Robotics on 4/9/2018.
 */

public class NeoMotor {
    DcMotor motor;
    public NeoMotor(LinearOpMode opMode, String name) {
        this.motor = opMode.hardwareMap.get(DcMotor.class, name);
    }
}
