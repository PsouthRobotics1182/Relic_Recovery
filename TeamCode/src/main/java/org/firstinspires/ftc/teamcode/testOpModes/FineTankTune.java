package org.firstinspires.ftc.teamcode.testOpModes;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import lib.fine.core.PidUdpReceiver;
import lib.fine.systems.FineBot;
import lib.fine.systems.SpeedyBot;

/**
 * Created by drew on 11/23/17.
 */
@TeleOp(name = "TeleTune")
public class FineTankTune extends LinearOpMode {
    private double p,i,d;

    private PidUdpReceiver pidUdpReceiver;
    @Override
    public void runOpMode() throws InterruptedException {
        SpeedyBot robot = new SpeedyBot(this, DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pidUdpReceiver = new PidUdpReceiver();
        pidUdpReceiver.beginListening();

        telemetry.addData("Ready", null);
        telemetry.update();
        waitForStart();
        double crossPower;
        double correction = 0;
        boolean release = false;
        while (opModeIsActive()) {
            updateCoefficients();

            correction = (gamepad1.a) ? robot.drive.imu.alignTune(0, p, i, d) : 0;

            if (gamepad1.x)
                robot.drive.imu.resetPID();
            if (gamepad1.b)
                robot.drive.imu.resetAngle();

            robot.drive.setLeftPower(Range.clip(-gamepad1.left_stick_y-correction, -0.5, 0.5));
            robot.drive.setRightPower(Range.clip(-gamepad1.right_stick_y+correction, -0.5, 0.5));

            crossPower = (gamepad1.left_trigger > 0)  ? -gamepad1.left_trigger
                       : (gamepad1.right_trigger > 0) ? -gamepad1.right_trigger
                       : 0;

            robot.drive.setCrossPower(crossPower);

            /*
            if (gamepad1.right_bumper)
                robot.reversePullBois.out();
            else if (gamepad1.left_bumper)
                robot.reversePullBois.in();
            else
                robot.reversePullBois.stop();
            */

            //robot.reversePullBois.in();



            telemetry.addData("p", + p + " i: " + i + " d: " + d);
            telemetry.update();
        }
        pidUdpReceiver.shutdown();
    }
    private void updateCoefficients()
    {
        p = pidUdpReceiver.getP();
        i = pidUdpReceiver.getI();
        d = pidUdpReceiver.getD();
    }
}
