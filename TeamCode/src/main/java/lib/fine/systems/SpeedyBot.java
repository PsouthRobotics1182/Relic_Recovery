package lib.fine.systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import lib.fine.core.FineIMU;

/**
 * Created by drew on 11/24/17.
 */

public class SpeedyBot {
    public FineSlide drive;
    public Flipper lift;
    public SuckyBois suckyBois;
    public JuulHittererer juulHittererer;
    public FishinPol pol;

    private LinearOpMode opMode;

    public static final int DRIVE_OFF_DISTANCE = 760;

    //runmode is whether to use encoders or not
    public SpeedyBot(LinearOpMode opMode, DcMotor.RunMode mode) {
        drive = new FineSlide(opMode, mode);
        lift = new Flipper(opMode);
        suckyBois = new SuckyBois(opMode);
        juulHittererer = new JuulHittererer(opMode);
        pol = new FishinPol(opMode, DcMotor.RunMode.RUN_USING_ENCODER);
        drive.setMode(FineIMU.Mode.OFF_PAD);

        this.opMode = opMode;
    }

    public void depositGlyph() {
        suckyBois.setPower(-1);

        drive.resetEncoders();
        while (drive.drivingForwardConserv(180, 1, 0)) {
            opMode.idle();
        }
        drive.stop();

        drive.driveBackward(100, 1);

        suckyBois.setPower(0);
    }

    public void sleep(int ms) {
        ElapsedTime runTime = new ElapsedTime();
        runTime.reset();

        while (runTime.milliseconds() < ms && opMode.opModeIsActive()) {
            opMode.telemetry.addData("Time", runTime.milliseconds() + "/" + ms);
            opMode.telemetry.update();
            opMode.idle();
        }

    }

    public void addTelemetry() {
        drive.addTelemetry();
        lift.addTelemetry();
    }
}
