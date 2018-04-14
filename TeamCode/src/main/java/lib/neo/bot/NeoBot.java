package lib.neo.bot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import lib.neo.drive.core.NeoDrive;

public class NeoBot {
    LinearOpMode opMode;
    NeoDrive driveTrain;
    public NeoBot(LinearOpMode opMode) {
        this.opMode = opMode;
    }
}
