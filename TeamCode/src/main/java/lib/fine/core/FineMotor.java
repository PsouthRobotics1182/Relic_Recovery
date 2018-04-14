package lib.fine.core;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by drew on 11/23/17.
 */

public class FineMotor {

    private DcMotor motor;
    //stores postion so it can be easily reset without setting mode to STOP_AND_RESET_ENCODERS
    private int zeroPosition = 0;
    private DcMotor.RunMode mode = DcMotor.RunMode.RUN_USING_ENCODER;
    private LinearOpMode opMode;
    private String name;

    /**
     * wraps DcMotor
     * @param opMode opmode which is creating the DcMotor
     * @param name name of the DcMotor in the XML
     */
    public FineMotor (LinearOpMode opMode, String name) {
        this.opMode = opMode;
        this.name = name;
        motor = opMode.hardwareMap.get(DcMotor.class, name);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     *
     * @return ticks per a revolution of the output shaft
     */
    public double getTicksPerRev() {
        return motor.getMotorType().getTicksPerRev();
    }

    /**
     *
     * @return maximum rotation speed in ticks per second
     */
    public double getTicksPerSecond() {
        return motor.getMotorType().getAchieveableMaxTicksPerSecondRounded();
    }

    /**
     *
     * @param power voltage to send to motor scale 0-1
     */
    public void setPower(double power) {
        motor.setPower(power);
    }

    /**
     *
     * @return power to the motor
     */
    public double getPower() {
        return motor.getPower();
    }

    /**
     * resets encoder so {link #getCurrentPosition()} would return zero
     */
    public void resetEncoder() {
        zeroPosition = motor.getCurrentPosition();
    }

    /**
     *
     * @return encoder position scaled by {link zeroPosition} to allow for resetting of encoder position
     */
    public int getCurrentPosition() {
        return motor.getCurrentPosition() - zeroPosition;
    }

    /**
     *
     * @return position of encoder from initialization, not affected by {link #resetEncoder()}
     */
    public int getAbsolutePosition() {
        return motor.getCurrentPosition();
    }

    /**
     * changes direction of the motor
     * @param direction direction of motor
     */
    public void setDirection(DcMotorSimple.Direction direction) {
        motor.setDirection(direction);
    }

    /**
     *
     * @param mode with or without encoders
     */
    public void setMode(DcMotor.RunMode mode) {
        this.mode = mode;
        motor.setMode(mode);
    }

    /**
     * crap, don't use it
     * @param pos crap, don't use it
     */
    public void setTargetPosition(int pos) {
        motor.setTargetPosition(pos);
    }

    public void addTelemetry() {
        opMode.telemetry.addData(name + " power", motor.getPower());
        opMode.telemetry.addData(name + " position", getCurrentPosition());
        opMode.telemetry.addData(name + " absolute position", getAbsolutePosition());
    }

    /**
     * to prevent the motor from spinning to to let it go
     * @param behav FLOAT means free spinning
     */
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behav) {
        motor.setZeroPowerBehavior(behav);
    }

}
