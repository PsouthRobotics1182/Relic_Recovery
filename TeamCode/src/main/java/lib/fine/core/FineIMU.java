package lib.fine.core;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 *
 * This class is designed to wrap the built-in BN055 IMU within the rev hub with PID and angle resetting features
 */
public class FineIMU {

    /**
     * This enum is used to manage whether the robot is on or off the balancing pad
     * to allow for 2 different PID tunes, since the pad is low traction
     */
    public enum Mode {
        ON_PAD,
        OFF_PAD
    }
    private Mode mode = Mode.ON_PAD;

    private BNO055IMU imu;
    private Orientation angles;
    private ElapsedTime runTime;
    private LinearOpMode opMode;
    private String name;

    /**
     * variables to manage PID's math
     */
    private double prevError = 0;
    private double integral = 0;
    private double zeroPos = 0;

    /**
     * PID tuning variable
     * DO NOT directly call this, use {@link #kP() kp} method set instead
     */
    private static final double kP_OFF_PAD = 0.041;//0.031
    private static final double kI_OFF_PAD = 0.00;//0.00015
    private static final double kD_OFF_PAD = 0.1;//0.21

    private static final double kP_ON_PAD = 0.02;
    private static final double kI_ON_PAD = 0;
    private static final double kD_ON_PAD = 0.24;

    /**
     * makes IMU which can do PID and reset angle easily
     * @param opMode the opmode that is using the IMU
     * @param name name in the xml
     */
    public FineIMU(LinearOpMode opMode, String name) {
        this.opMode = opMode;
        this.name = name;

        runTime = new ElapsedTime();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = opMode.hardwareMap.get(BNO055IMU.class, name);
        imu.initialize(parameters);                                    //how often to update values
        imu.startAccelerationIntegration(new Position(), new Velocity(), 100);
    }

    /**
     * changes from on balancing pad tune to off pad tune
     * @param mode the desired PID tune to use
     */
    public void setMode(Mode mode) {
        this.mode = mode;
    }

    /**
     * call this method to get the p constant based on the PID tune mode
     * @return P constant for PID loop
     */
    private double kP() {
        return (mode == Mode.ON_PAD) ? kP_ON_PAD : kP_OFF_PAD;
    }

    private double kI() {
        return (mode == Mode.ON_PAD) ? kI_ON_PAD : kI_OFF_PAD;
    }

    private double kD() {
        return (mode == Mode.ON_PAD) ? kD_ON_PAD : kD_OFF_PAD;
    }

    /**
     *
     * @return heading shifted based on how it was reset
     */
    public double getHeading() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle - zeroPos;
    }

    /**
     *
     * @return rate of rotation given by the IMU
     */
    public double getRate() {
        return imu.getAngularVelocity().zRotationRate;
    }

    /**
     * uses acceleromet within the imu to get the angle of rotation relative to the ground
     * @return array with values corresponding to angles off level
     */
    public double[] getLevelness() {
        double[] levels = new double[2];
        levels[0] = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle;
        levels[1] = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).thirdAngle;
        return levels;
    }

    /**
     * must calibrate IMU for this to work
     * @return acceleration value from IMU
     */
    public Acceleration getAcceleration() {
        return imu.getAcceleration();
    }

    /**
     * must calibrate IMU for this to work
     * @return velocity value from IMU
     */
    public Velocity getVelocity() {
        return imu.getVelocity();
    }

    /**
     * must calibrate IMU for this to work
     * @return position value from IMU
     */
    public Position getPosition() {
        return imu.getPosition();
    }

    /**
     * resets PID variables so it can be used for a separate process
     */
    public void resetPID() {
        prevError = 0;
        integral = 0;
    }

    /**
     * resets angle so following call to {@link #getHeading()}
     */
    public void resetAngle() {
        zeroPos = getHeading();
    }

    /**
     * uses the PID algorithm to calculate desired power to get to the desired heading
     * @param angle angle to align to in degrees
     * @return the power to get to the angle
     */
    public double align (double angle) {
        if (prevError == 0) runTime.reset();
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        final double heading = getHeading();
        final double error = angle-heading;
        integral = integral + error * kP() *runTime.milliseconds();
        final double deravitive = (error - prevError)/runTime.milliseconds();
        final double correction = error * kP() + deravitive * kD() + integral * kI();
        //correction = Range.clip(correction, 0, 0.3);
        prevError = error;
        runTime.reset();
        return correction;
    }

    /**
     * copy of {link #align(double angle)} but takes PID constants, designed to be used to tune the system
     * @param angle desired angle to align to
     * @param p P tune constant
     * @param i I tune constant
     * @param d D tune constant
     * @return th power to get to the angle
     */
    public double alignTune (double angle, double p, double i, double d) {
        if (prevError == 0) runTime.reset();
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        final double heading = getHeading();
        final double error = angle-heading;
        integral = integral + error * p * runTime.milliseconds();
        final double deravitive = (error - prevError)/runTime.milliseconds();
        final double correction = error * p + deravitive * d + integral * i;
        //correction = Range.clip(correction, 0, 0.3);
        prevError = error;
        runTime.reset();
        return correction;
    }

    public void addTelemetry() {
        opMode.telemetry.addData(name + " heading", getHeading());
        opMode.telemetry.addData(name + " heading", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        opMode.telemetry.addData(name + " rate", imu.getAngularVelocity().zRotationRate);
    }

}
