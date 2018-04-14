package lib.fine.core;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;

/**
 * Created by drew on 3/22/18.
 */

public class LimitSwitch {

    private LinearOpMode opMode;

    private DigitalChannel limit;

    /**
     * creates an object to tell if a switch is pressed
     * @param name name of the switch in XML
     * @param opMode opmode creating the switch
     */
    public LimitSwitch(String name, LinearOpMode opMode) {
        limit = opMode.hardwareMap.get(DigitalChannel.class, name);
        limit.setMode(DigitalChannel.Mode.INPUT);
        this.opMode = opMode;
    }

    /**
     * manages the conversion between state and if it pressed
     * @return
     */
    public boolean isPressed() {
        return !limit.getState(); //true is not pressed

    }
}

