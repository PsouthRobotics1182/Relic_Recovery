package lib.fine.systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import lib.fine.core.LimitSwitch;

/**
 * Created by drew on 11/24/17.
 */


//Code for flipiing metal plate
public class Flipper {
    //private FineMotor lift;
    private Servo flipper;
    private Servo servo12;

    private CRServo topLeft;
    private CRServo topRight;
    private CRServo bottomRight;
    private CRServo bottomLeft;

    private LimitSwitch topLimit;
    private LimitSwitch bottomLimit;

    private LinearOpMode opMode;
    public static double VERTICAL = 0.45;
    public static double HORIZONTAL = 0.8;
    public static double INTAKE = 1;
    public static double OVER_DUMP = 0;


    public Flipper(LinearOpMode opMode) {
        flipper = opMode.hardwareMap.get(Servo.class, "flip");
        servo12 = opMode.hardwareMap.get(Servo.class, "servo12");

        topLeft = opMode.hardwareMap.get(CRServo.class, "topLeft");
        topRight = opMode.hardwareMap.get(CRServo.class, "topRight");
        bottomRight = opMode.hardwareMap.get(CRServo.class, "bottomRight");
        bottomLeft = opMode.hardwareMap.get(CRServo.class, "bottomLeft");

        topLimit = new LimitSwitch("topSwitch", opMode);
        bottomLimit = new LimitSwitch("bottomSwitch", opMode);

        this.opMode = opMode;
        flip(1);
        block();

    }
    public void lift(double power) {
        if (power >= 0 && atTop())
            power = 0;
        else if (power <= 0 && atBottom())
            power = 0;
        setPower(power);

    }

    private void setPower(double power) {
        topLeft.setPower(power);
        topRight.setPower(-power);
        bottomLeft.setPower(power);
        bottomRight.setPower(-power);
    }

    public double getPower() {
        double max1 = Math.max(topLeft.getPower(), topRight.getPower());
        double max2 = Math.max(bottomLeft.getPower(), bottomRight.getPower());
        return Math.max(max1, max2);
    }

    public void block() {
        servo12.setPosition(0.5);
    }
    public void unBlock() {
        servo12.setPosition(1);
    }
    public void flip (double flipiness) {
        flipper.setPosition(flipiness);
    }
    public boolean atTop() {
        return topLimit.isPressed();
    }
    public boolean atBottom() {
        return bottomLimit.isPressed();
    }
    public void addTelemetry() {
        //lift.addTelemetry();
    }

}
