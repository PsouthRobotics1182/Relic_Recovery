package org.firstinspires.ftc.teamcode.oldOpModes;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import lib.fine.systems.FineBot;

/**
 * Created by drew on 11/23/17.
 */
@Disabled

@TeleOp(name = "TeleOperations v3.45423")
public class FineTele extends LinearOpMode {
    FineBot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new FineBot(this, DcMotor.RunMode.RUN_WITHOUT_ENCODER, Color.RED);

        telemetry.addData("Ready", null);
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            driveTrain();
            bully();
            lift();
            if (gamepad2.a) {
                robot.juulHittererer.hitRed();
            }
        }
    }

    private void driveTrain() {
        double correction = (gamepad1.a) ? robot.drive.imu.align(0) : 0;

        telemetry.addData("Correction", correction);

        robot.drive.setLeftPower(-gamepad1.left_stick_y-correction);
        robot.drive.setRightPower(-gamepad1.right_stick_y+correction);

        double crossPower = (gamepad1.left_trigger > 0) ? -gamepad1.left_trigger : (gamepad1.right_trigger > 0) ? gamepad1.right_trigger : 0;

        robot.drive.setCrossPower(crossPower);
    }


    final double sensitivity = 0.3;

    private void lift() {
        robot.lift.lift(-gamepad2.right_stick_y);

        if (gamepad2.left_trigger > sensitivity)
            robot.lift.grabBottom(0.35);
        else if (gamepad2.right_trigger > sensitivity)
            robot.lift.grabBottom(1);

        if (gamepad2.left_bumper)
            robot.lift.grabTop(0.2);
        else if (gamepad2.right_bumper)
            robot.lift.grabTop(1);
    }

    private void bully() {
        robot.reversePullBois.in();
        robot.reversePullBois.powerLift((gamepad1.left_bumper) ? 1 : (gamepad1.right_bumper) ? -1 : 0);

    }
}
