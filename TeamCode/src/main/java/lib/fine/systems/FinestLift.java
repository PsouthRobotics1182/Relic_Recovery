package lib.fine.systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import lib.fine.core.FineMotor;

/**
 * Created by drew on 11/24/17.
 */
@Deprecated
public class FinestLift {
    public FineMotor leftMotor;
    public FineMotor rightMotor;

    private Servo leftGrasper;
    private Servo rightGrasper;
    private Servo leftGrasperTop;
    private Servo rightGrasperTop;
    private LinearOpMode opMode;

    private static final double LEFT_UNGRAB_BOTTOM = 1;
    private static final double RIGHT_UNGRAB_BOTTOM = 0.7;//0.45
    private static final double LEFT_GRAB_BOTTOM = 0;
    private static final double RIGHT_GRAB_BOTTOM = 1;
    private static final double LEFT_GRAB_TOP = 0;
    private static final double RIGHT_GRAB_TOP = 0.8;
    private static final double LEFT_UNGRAB_TOP = 0.87;//0.85
    private static final double RIGHT_UNGRAB_TOP = 0.1;
    public static final double STAY_POWER = 0.0011;

    public FinestLift(LinearOpMode opMode, DcMotor.RunMode mode) {
        leftMotor = new FineMotor(opMode, "leftMotor");
        rightMotor = new FineMotor(opMode, "rightMotor");

        rightGrasper = opMode.hardwareMap.get(Servo.class, "leftGrasp");
        leftGrasper = opMode.hardwareMap.get(Servo.class, "rightGrasp");
        leftGrasperTop = opMode.hardwareMap.get(Servo.class, "leftGraspTop");
        rightGrasperTop = opMode.hardwareMap.get(Servo.class, "rightGraspTop");

        leftMotor.setMode(mode);
        rightMotor.setMode(mode);

        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        this.opMode = opMode;

        grabBottom(0.3);
        grabTop(0.1);
    }

    public void lift(double power) {
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }

    public void grabBottom(double grabiness) {
        leftGrasper.setPosition(LEFT_GRAB_BOTTOM * grabiness + LEFT_UNGRAB_BOTTOM * (1-grabiness));
        rightGrasper.setPosition(RIGHT_GRAB_BOTTOM * grabiness + RIGHT_UNGRAB_BOTTOM * (1-grabiness));
        //leftGrasperTop.setPosition(LEFT_GRAB_TOP * grabiness + LEFT_UNGRAB_TOP * (1-grabiness));
        //rightGrasperTop.setPosition(RIGHT_GRAB_TOP * grabiness + RIGHT_UNGRAB_TOP * (1-grabiness));
    }
    public void grabTop(double grabiness) {
        //leftGrasper.setPosition(LEFT_GRAB_BOTTOM * grabiness + LEFT_UNGRAB * (1-grabiness));
        //rightGrasper.setPosition(RIGHT_GRAB * grabiness + RIGHT_UNGRAB * (1-grabiness));
        leftGrasperTop.setPosition(LEFT_GRAB_TOP * grabiness + LEFT_UNGRAB_TOP * (1-grabiness));
        rightGrasperTop.setPosition(RIGHT_GRAB_TOP * grabiness + RIGHT_UNGRAB_TOP * (1-grabiness));
    }
    public void leftSwingBottom(double grabiness) {
        leftGrasper.setPosition(LEFT_GRAB_BOTTOM * grabiness + LEFT_UNGRAB_BOTTOM * (1-grabiness));
        rightGrasper.setPosition(RIGHT_UNGRAB_BOTTOM);
    }
    public void rightSwingBottom(double grabiness) {
        leftGrasper.setPosition(LEFT_UNGRAB_BOTTOM);
        rightGrasper.setPosition(RIGHT_GRAB_BOTTOM * grabiness + RIGHT_UNGRAB_BOTTOM * (1-grabiness));
    }

    public void dab() {
        leftMotor.setPower(1);
        opMode.sleep(700);
        leftGrasperTop.setPosition(LEFT_GRAB_BOTTOM);
        rightGrasperTop.setPosition(RIGHT_UNGRAB_BOTTOM);
    }
    public void addTelemetry() {
        leftMotor.addTelemetry();
        opMode.telemetry.addData("left servo", leftGrasper.getPosition());
        opMode.telemetry.addData("right servo", rightGrasper.getPosition());
    }
}
