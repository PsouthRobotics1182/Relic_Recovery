package lib.fine.systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import lib.fine.core.FineMotor;

/**
 * Created by drew on 11/24/17.
 */

public class FishinPol {
    private FineMotor extender;
    private Servo rotatar;
    private Servo grabbar;

    private LinearOpMode opMode;

    public static final double GRAB = 0;
    public static final double UNGRAB = 1;
    public static final double DOWN = 1;
    public static final double UP = 0;


    public FishinPol(LinearOpMode opMode, DcMotor.RunMode mode) {
        extender = new FineMotor(opMode, "pol");

        grabbar = opMode.hardwareMap.get(Servo.class, "grabber");
        rotatar = opMode.hardwareMap.get(Servo.class, "relicRot");

        rotatar.setPosition(DOWN);
        grabbar.setPosition(UNGRAB);
        extender.setMode(mode);
        extender.setDirection(DcMotorSimple.Direction.FORWARD);
        this.opMode = opMode;
    }

    public void setPower(double power) {
        extender.setPower(-power);
    }

    public void grab(double grabiness) {
        grabbar.setPosition(GRAB * grabiness + UNGRAB * (1-grabiness));
    }

    public void rotate(double upness) {
        rotatar.setPosition(UP * upness + DOWN * (1-upness));
    }

    public void addTelemetry() {
        extender.addTelemetry();
    }
}
