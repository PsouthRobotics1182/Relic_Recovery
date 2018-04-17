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

    public void depositGlyphNGyro() {
        suckyBois.setPower(-1);

        sleep(1500);
        drive.driveBackward(80, 1);

        drive.encoderTurn(180/2, 1);

        drive.resetEncoders();
        while (drive.drivingBackwardConsv(180, 0.5)) {
            opMode.idle();
        }
        drive.stop();

        drive.resetAngle();
        drive.resetEncoders();
        while (drive.drivingForwardNoGyro(100, 0.5)) {
            opMode.idle();
        }
        drive.stop();

        suckyBois.setPower(0);
    }
    public void depositGlyph() {
        drive.resetAngle();
        //suckyBois.setPower(-1);

        lift.flip(Flipper.VERTICAL);

        drive.resetAngle();
        drive.resetEncoders();
        while (drive.drivingForwardNoGyro(180, 1)) {
            opMode.idle();
        }
        drive.stop();

        suckyBois.setPower(-1);
        sleep(1000);
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
