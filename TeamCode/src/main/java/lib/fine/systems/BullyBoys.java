package lib.fine.systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;

import lib.fine.core.FineMotor;

/**
 * Created by Robotics on 12/4/2017.
 */
@Deprecated
public class BullyBoys {
    private CRServo left;
    private CRServo right;
    public FineMotor bullyBoiReverseLowerer;
    final static int UP_POS = -100;

    public BullyBoys(LinearOpMode opMode) {
        left = opMode.hardwareMap.get(CRServo.class, "left");
        right = opMode.hardwareMap.get(CRServo.class, "right");
        bullyBoiReverseLowerer = new FineMotor(opMode, "lifter");
        bullyBoiReverseLowerer.setPower(0);
        //bullyBoiReverseLowerer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void out() {
        left.setPower(1);
        right.setPower(-1);
    }
    public void in () {
        left.setPower(-1);
        right.setPower(1);
    }
    public void stop() {
        left.setPower(0);
        right.setPower(0);
        bullyBoiReverseLowerer.setPower(0);
    }
    public void up(double power) {
        bullyBoiReverseLowerer.setPower(-power);
    }
    public void down(double power) {
        bullyBoiReverseLowerer.setPower(power);
    }
    public void powerLift(double power) {
        bullyBoiReverseLowerer.setPower(-power);
    }

/*    public void addTelemetry() {
        leftMotor.addTelemetry();
        opMode.telemetry.addData("left servo", leftGrasper.getPositionMin());
        opMode.telemetry.addData("right servo", rightGrasper.getPositionMin());
    }*/
}
