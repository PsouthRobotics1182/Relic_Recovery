package org.firstinspires.ftc.teamcode.oldOpModes;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import lib.fine.core.FineIMU;
import lib.fine.systems.FineBot;
import lib.fine.vision.BetterVuforia;

/**
 * Created by drew on 11/25/17.
 */
@Disabled
@Autonomous(name = "Strawberry Juul V2.386N")
public class RedJuulNear extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        FineBot robot = new FineBot(this, DcMotor.RunMode.RUN_USING_ENCODER, Color.RED);
        BetterVuforia vuforia = new BetterVuforia(this, true);
        ElapsedTime runTime = new ElapsedTime();
        vuforia.activate();

        telemetry.addData("Ready", null);
        telemetry.update();

        waitForStart();
        runTime.reset();

        RelicRecoveryVuMark column = vuforia.getColumn();
        robot.lift.grabBottom(1);
        robot.lift.grabTop(1);
        robot.sleep(1000);
        robot.lift.lift(1);
        robot.sleep(500);
        robot.lift.lift(0);

        robot.reversePullBois.powerLift(0.5);
        robot.sleep(500);
        robot.reversePullBois.powerLift(0);

        robot.juulHittererer.hitRed();

        robot.sleep(500);
        robot.reversePullBois.powerLift(-0.8);
        robot.sleep(500);
        robot.reversePullBois.powerLift(0);

        robot.drive.resetEncoders();
        //double power = 0;
        robot.drive.strafeLeft(780, 1);
        robot.drive.stop();

        column = vuforia.getColumn();

        robot.drive.imu.setMode(FineIMU.Mode.ON_PAD);
        //robot.sleep(5000);

        robot.drive.resetEncoders();
        runTime.reset();
        while (robot.drive.drivingForwardConserv(720, 0.6, 0)) {
            column = (column == RelicRecoveryVuMark.UNKNOWN) ? vuforia.getColumn() : column;
        }
        robot.drive.stop();

        robot.drive.imu.setMode(FineIMU.Mode.OFF_PAD);

        robot.drive.setAngleTolerence(2);

        column = (column == RelicRecoveryVuMark.UNKNOWN) ? vuforia.getColumn() : column;

        telemetry.addData("Column", column);
        telemetry.update();

        robot.drive.rotateNoReser(-90, 0.7);

        robot.sleep(500);
        robot.drive.rotate(-90, 0.5);
        //sleep(5000);
        int mmBase = 1010;
        int mmStrafe = mmBase;

       /*
         TODO: If this is to be used, I'll rewrite it as switch-case like the others. ~David
         if (column == RelicRecoveryVuMark.UNKNOWN)
            column = RelicRecoveryVuMark.RIGHT;
        if (column == RelicRecoveryVuMark.RIGHT)
            mmStrafe = mmBase;
        else if (column == RelicRecoveryVuMark.CENTER)
            mmStrafe = mmBase + 190;
        else if (column == RelicRecoveryVuMark.LEFT)
            mmStrafe = mmBase + 190 + 190;
*/

        //robot.lift.leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.drive.strafeRange(mmStrafe, 0.8);

        robot.drive.resetEncoders();
        while (robot.drive.drivingForwardConserv(260, 1, 0)) {
            idle();
        }
        robot.drive.stop();

        robot.drive.setAngleTolerence(3);

        int theta = 0;

        switch(column) {
            case LEFT:
                theta = 30;
                break;
            case UNKNOWN:
            case CENTER:
                theta = 20;
        }

        robot.drive.rotate(theta, 0.5);

        robot.drive.resetAngle();



        //robot.drive.driveBackward(100, 1);
/*
        robot.drive.resetEncoders();
        */
        robot.drive.resetEncoders();
        while (robot.drive.drivingForwardNoGyro(120, 0.5)) {
            idle();
        }
        robot.drive.stop();
        //robot.drive.driveForward(120, 1);
        robot.lift.grabBottom(0);
        robot.drive.driveBackward(200, 1);
/*
        robot.reversePullBois.powerLift(0.5);
        robot.sleep(500);
        robot.reversePullBois.powerLift(0);
        robot.lift.grabTop(0.1);
        robot.lift.grabBottom(0.3);
        //  robot.fishinPol.rotate(0);
        robot.sleep(500);*/
    }
}
