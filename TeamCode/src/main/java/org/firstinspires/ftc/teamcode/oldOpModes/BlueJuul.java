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

@Autonomous(name = "BlueBerry Juul V2.266")
public class BlueJuul extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        FineBot robot = new FineBot(this, DcMotor.RunMode.RUN_USING_ENCODER, Color.BLUE);
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
        robot.sleep(800);
        robot.lift.lift(0);

        robot.reversePullBois.powerLift(0.5);
        robot.sleep(500);
        robot.reversePullBois.powerLift(0);

        robot.juulHittererer.hitBlue();

        robot.sleep(500);
        robot.reversePullBois.powerLift(-0.8);
        robot.sleep(500);
        robot.reversePullBois.powerLift(0);

        robot.drive.resetEncoders();
        //double power = 0;
        robot.drive.strafeLeft(780, 1);
        robot.drive.stop();

        column = vuforia.getColumn();

        robot.drive.imu.setMode(FineIMU.Mode.OFF_PAD);

        robot.drive.resetEncoders();
        runTime.reset();
        while (robot.drive.drivingBackwardConsv(810, 0.6)) {
            idle();
            if (column == RelicRecoveryVuMark.UNKNOWN)
                column = vuforia.getColumn();
        }
        robot.drive.stop();

        robot.drive.imu.setMode(FineIMU.Mode.OFF_PAD);

        robot.drive.resetEncoders();
        while (robot.drive.drivingForwardConserv(150, 1, -90)) {
            idle();
        }
        robot.drive.stop();
        robot.drive.setAngleTolerence(3);

        robot.drive.rotateNoReser(-90, 0.7);

        telemetry.addData("Column", column);
        telemetry.update();
        int mmBase = 450;
        int mmStrafe = mmBase;

        switch(column) {
            case CENTER:
                mmStrafe = mmBase + 190;
                break;
            case UNKNOWN: // Fall-Through to Right
                column = RelicRecoveryVuMark.RIGHT;
            case RIGHT:
                mmStrafe = mmBase + 190 + 190;
                break;
            case LEFT:
                mmStrafe = mmBase;
                break;
        }

        /*robot.lift.lift(1);
        robot.sleep(500);
        robot.lift.lift(0);*/

        robot.drive.resetEncoders();
        runTime.reset();
        while (robot.drive.strafingRange(mmStrafe, 0.8)&& runTime.milliseconds() < 5000 ){
            idle();
        }
        robot.drive.stop();

        //robot.drive.rotate(-90, 0.7);

        //robot.drive.rotate(0, 0.5);
        //robo

        runTime.reset();
        while (robot.drive.rotating(90, 0.5) && runTime.milliseconds() < 10000) {
            column = (column == RelicRecoveryVuMark.UNKNOWN) ? vuforia.getColumn() : column;
            idle();
        }
        robot.drive.stop();
        runTime.reset();
        while (robot.drive.rotating(90, 0.5) && runTime.milliseconds() < 5000) {
            column = (column == RelicRecoveryVuMark.UNKNOWN) ? vuforia.getColumn() : column;
            idle();
        }
        robot.drive.stop();
        robot.drive.imu.resetAngle();

        robot.lift.lift(-1);
        robot.sleep(300);
        robot.lift.lift(0);
        robot.lift.grabBottom(0);
        robot.sleep(500);

        robot.lift.lift(1);
        robot.sleep(700);
        robot.lift.lift(0);
        robot.drive.resetEncoders();
        while (robot.drive.drivingForwardConserv(120, 0.8, 0) && runTime.milliseconds() < 5000) {
            idle();
        }
        robot.drive.stop();

        robot.drive.driveBackward(100, 1);

        robot.lift.grabBottom(0.35);

        robot.sleep(1000);
        robot.drive.resetEncoders();

        while (robot.drive.drivingForwardConserv(120, 1, 0)) {
            idle();
        }

        robot.drive.driveBackward(110, 1);

        /*robot.reversePullBois.powerLift(0.5);
        robot.sleep(500);
        robot.reversePullBois.powerLift(0);*/
        robot.lift.grabTop(0.1);
        //robot.fishinPol.rotate(0);
        robot.sleep(500);
    }
}
