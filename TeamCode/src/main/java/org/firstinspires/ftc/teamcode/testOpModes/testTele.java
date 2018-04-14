package org.firstinspires.ftc.teamcode.testOpModes;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import lib.fine.systems.Flipper;
import lib.fine.systems.SpeedyBot;

/**
 * tele-op for the new robot
 */
@TeleOp(name = "Test v4.45523")
public class testTele extends LinearOpMode {
    SpeedyBot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new SpeedyBot(this, DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Ready", null);
        telemetry.update();

        waitForStart();
        telemetry.clearAll();

        while (opModeIsActive()) {

            driveTrain();
           /* intake();
            lift();
            boot();
            fishing();*/

            if (gamepad2.b)
                robot.juulHittererer.hitRed();
            if (gamepad2.x)
                robot.juulHittererer.hitBlue();

            telemetry.update();
        }
    }

    private void driveTrain() {
        //A button auto aligns the robot to square with the cryptobox
        double correction = (gamepad1.a) ? robot.drive.imu.align(0) : 0;

        robot.drive.setLeftPower(-gamepad1.left_stick_y-correction);
        robot.drive.setRightPower(-gamepad1.right_stick_y+correction);

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
        if (robot.suckyBois.getPower() > 0.1)
            robot.suckyBois.boot(0.5);
        if (gamepad2.a)
            robot.suckyBois.boot(1);
        else if (gamepad2.b)
            robot.suckyBois.boot(0.5);
    }

    private void fishing() {
        robot.pol.setPower(-gamepad2.left_stick_y);
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
