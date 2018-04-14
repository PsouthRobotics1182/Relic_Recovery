package org.firstinspires.ftc.teamcode.currentOpModes;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import lib.fine.systems.Flipper;
import lib.fine.systems.SpeedyBot;

/**
 * tele-op for the new robot
 */
@TeleOp(name = "TeleOperations v4.45523")
public class NewTeleOp extends LinearOpMode {
    SpeedyBot robot;
    ElapsedTime timer;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new SpeedyBot(this, DcMotor.RunMode.RUN_WITHOUT_ENCODER);

         timer = new ElapsedTime();

        telemetry.addData("Ready", null);
        telemetry.update();

        waitForStart();
        telemetry.clearAll();
        int fishing = 1;
        int glyphs = 0;
        int mode = glyphs;
        while (opModeIsActive()) {

            if (gamepad2.x) {
                mode = fishing;
                robot.pol.rotate(0.5);
                robot.pol.grab(1);
            } else if (gamepad2.left_bumper)
                mode = glyphs;
            driveTrain();
            if (mode == glyphs) {
                telemetry.addData("mode:", "glyphs");
                intake();
                lift();
                boot();
            }else if (mode == fishing) {
                robot.lift.unBlock();
                telemetry.addData("mode:", "relic");
                fishing();
            }

            telemetry.update();
        }
    }


    private void driveTrain() {

        if (gamepad1.x)
            robot.lift.block();
        else if (gamepad1.b)
            robot.lift.unBlock();

        if (gamepad1.dpad_left)
            robot.suckyBois.boot(0);
        else if (gamepad1.dpad_right)
            robot.suckyBois.boot(1);
        //A button auto aligns the robot to square with the cryptobox
        //double correction = (gamepad1.a) ? robot.drive.imu.align(0) : 0;

        double speed = 0.45;
        if (gamepad1.dpad_up)
            robot.drive.setLeftPower(speed);
        else if (gamepad1.dpad_down)
            robot.drive.setLeftPower(-speed);
        else
            robot.drive.setLeftPower(-gamepad1.left_stick_y);
        if (gamepad1.y)
            robot.drive.setRightPower(speed);
        else if (gamepad1.a)
            robot.drive.setRightPower(-speed);
        else
            robot.drive.setRightPower(-gamepad1.right_stick_y);

        //left and right triggers move robot side to side
        double crossPower = (gamepad1.left_trigger > 0) ? -gamepad1.left_trigger : (gamepad1.right_trigger > 0) ? gamepad1.right_trigger : 0;

        robot.drive.setCrossPower(crossPower);
    }

    private void intake() {
        if (gamepad2.left_bumper)
            robot.suckyBois.setLeftPower(-1);
        else
            robot.suckyBois.setLeftPower(gamepad2.left_trigger);

        if (gamepad2.right_bumper)
            robot.suckyBois.setRightPower(-1);
        else
            robot.suckyBois.setRightPower(gamepad2.right_trigger);
    }


    private void boot() {
        if (!robot.lift.atBottom() && timer.milliseconds() > 500) {
            robot.lift.block();
        } else if (robot.lift.atBottom()) {
            timer.reset();
            robot.lift.unBlock();
        }else
            robot.lift.unBlock();

        if (robot.suckyBois.getPower() > 0.1)
            robot.suckyBois.boot(0.5);
        if (gamepad2.a)
            robot.suckyBois.boot(1);
        else if (gamepad2.b)
            robot.suckyBois.boot(0.5);
    }

    double rotPos = 0;
    private void fishing() {
        robot.pol.setPower(-gamepad2.right_stick_y);
        if (gamepad2.a)
            robot.pol.grab(0);


        else if (gamepad2.b)
            robot.pol.grab(1);

        if (gamepad2.dpad_up)
            rotPos += 0.001;
        else if (gamepad2.dpad_down)
            rotPos -= 0.001;

        if (rotPos < 0) rotPos = 0;
        else if (rotPos > 1) rotPos = 1;
        if (gamepad2.dpad_right)
            rotPos = 0.5;
        else if (gamepad2.dpad_left)
            rotPos = 1;
        robot.pol.rotate(rotPos);

    }

    private void lift() {
        robot.lift.lift(-gamepad2.right_stick_y);//powers the 4 servos

        //DO NOT CHANGE ORDER OF THESE STATEMENTS
        //THEY ARE IN THIS ORDER TO PRIORITIZE DRIVER CONTROLS OVER AUTOMATED ONES


        if (gamepad2.dpad_up)
            robot.lift.flip(Flipper.VERTICAL);
        else if (gamepad2.dpad_right)
            robot.lift.flip(Flipper.HORIZONTAL);
        else if (gamepad2.dpad_left)
            robot.lift.flip(Flipper.OVER_DUMP);
        else if (gamepad2.dpad_down)
            robot.lift.flip(Flipper.INTAKE);
        else if (robot.lift.atBottom())
            robot.lift.flip(Flipper.INTAKE);
            //implied !robot.lift.atBottom() && since else
        else if (!robot.lift.atTop() && robot.lift.getPower() > 0.01)
            robot.lift.flip(Flipper.HORIZONTAL);
    }

}
