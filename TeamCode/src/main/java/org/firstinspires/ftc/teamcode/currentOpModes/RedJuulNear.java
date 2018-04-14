package org.firstinspires.ftc.teamcode.currentOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import lib.fine.systems.SpeedyBot;
import lib.fine.vision.BetterVuforia;

/**
 * Created by drew on 3/26/18.
 */
@Autonomous(name = "Strawberry Juul Near V4.396")
public class RedJuulNear extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SpeedyBot robot = new SpeedyBot(this, DcMotor.RunMode.RUN_USING_ENCODER);
        BetterVuforia vuforia = new BetterVuforia(this, true);
        ElapsedTime runTime = new ElapsedTime();
        telemetry.addData("Ready", "Press Start");
        telemetry.update();
        waitForStart();

        robot.juulHittererer.hitRed();

        robot.drive.strafeLeft(SpeedyBot.DRIVE_OFF_DISTANCE, 1);

        RelicRecoveryVuMark column = vuforia.getColumn();

        robot.drive.resetEncoders();
        while (robot.drive.drivingForwardConserv(720, 0.6, 0)) {
            idle();
        }
        robot.drive.stop();

        robot.drive.encoderTurn(-90, 1);

        int angle = -90;
        if (column == RelicRecoveryVuMark.UNKNOWN)
            column = RelicRecoveryVuMark.CENTER;
        else if (column == RelicRecoveryVuMark.LEFT)
            angle = -50;
        else if (column == RelicRecoveryVuMark.CENTER)
            angle = -70;
        else if (column == RelicRecoveryVuMark.RIGHT)
            angle = -90;

        robot.drive.rotate(angle, 1);

        robot.depositGlyph();

        robot.drive.rotate(-angle, 1);

    }
}
