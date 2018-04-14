package lib.fine.systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import lib.fine.core.FineMotor;

/**
 * Created by Robotics on 2/12/2018.
 */

public class SuckyBois {

    private LinearOpMode opMode;
    private FineMotor leftM;
    private FineMotor rightM;
    private Servo boot;

    public SuckyBois(LinearOpMode opMode) {
        this.opMode = opMode;
        leftM = new FineMotor(opMode, "leftM");
        leftM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftM.setDirection(DcMotorSimple.Direction.REVERSE);
        rightM = new FineMotor(opMode, "rightM");
        rightM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightM.setDirection(DcMotorSimple.Direction.FORWARD);

        boot = opMode.hardwareMap.get(Servo.class, "boot");
    }

    public void setLeftPower(double leftPower) {
        leftM.setPower(leftPower);
    }
    public void setRightPower(double rightPower) {
        rightM.setPower(rightPower);
    }
    public void setPower(double power) {
        setLeftPower(power);
        setRightPower(power);
    }

    public double getLeftPower() {
        return leftM.getPower();
    }
    public double getRightPower() {
        return rightM.getPower();
    }
    public double getPower() {
        return Math.max(leftM.getPower(), rightM.getPower());
    }
    public void boot(double bootiness) {
        boot.setPosition(bootiness);
    }
}
