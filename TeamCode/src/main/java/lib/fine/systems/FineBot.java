package lib.fine.systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import lib.fine.core.FineIMU;

/**
 * Created by drew on 11/24/17.
 */
@Deprecated
public class FineBot {
    public FineSlide drive;
    public FinestLift lift;
    public BullyBoys reversePullBois;
    public JuulHittererer juulHittererer;
    //public FishinPol fishinPol;
    private LinearOpMode opMode;
    private ElapsedTime runTime;

    private int alliance;

    public FineBot(LinearOpMode opMode, DcMotor.RunMode mode, int alliance) {
        drive = new FineSlide(opMode, mode);
        lift = new FinestLift(opMode, DcMotor.RunMode.RUN_USING_ENCODER);
        //fishinPol = new FishinPol(opMode, DcMotor.RunMode.RUN_USING_ENCODER);
        juulHittererer = new JuulHittererer(opMode);
        reversePullBois = new BullyBoys(opMode);

        runTime = new ElapsedTime();

        drive.setMode(FineIMU.Mode.OFF_PAD);
        //juulHittererer.up();
        //juulHittererer.middle();

        this.alliance = alliance;
        this.opMode = opMode;
    }


    private void resetTimer() {
        runTime.reset();
    }

    public void sleep(int ms) {
        resetTimer();
        while (runTime.milliseconds() < ms && opMode.opModeIsActive()) {
            opMode.telemetry.addData("Time", runTime.milliseconds() + "/" + ms);
            opMode.telemetry.update();
            opMode.idle();
        }
    }
    public void addTelemetry() {
        drive.addTelemetry();
        lift.addTelemetry();
        juulHittererer.addTelemetry();
    }
}
