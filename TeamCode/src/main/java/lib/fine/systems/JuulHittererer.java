package lib.fine.systems;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by drew on 11/24/17.
 */

public class JuulHittererer {
    public Servo juul;
    public Servo swingyBoi;
    public Servo pivot;
    private ColorSensor colorSensor;
    private LinearOpMode opMode;

    private static final int COLOR_CERTAINITY = 2;

    public JuulHittererer(LinearOpMode opMode) {
        this.opMode = opMode;
        colorSensor = opMode.hardwareMap.get(ColorSensor.class, "color");
        juul = opMode.hardwareMap.get(Servo.class, "juul");
        swingyBoi = opMode.hardwareMap.get(Servo.class, "swing");
        pivot = opMode.hardwareMap.get(Servo.class, "pivot");
        colorSensor.enableLed(true);
        home();
    }

    public int getColor() {
        int colorDiff = colorSensor.blue() - colorSensor.red();
        return (colorDiff > COLOR_CERTAINITY) ? Color.BLUE
             : (colorDiff < -COLOR_CERTAINITY) ? Color.RED
             : Color.BLACK;
    }

    public void left() {
        swingyBoi.setPosition(0);
    }
    public void right() {
        swingyBoi.setPosition(1);
    }

    public void home() {
        home1();
        opMode.sleep(500);
        home2();
        opMode.sleep(500);
        home3();

    }
    private void home1() {
        pivot.setPosition(0);
        swingyBoi.setPosition(0.107);
        juul.setPosition(0.2);
    }
    private void home2() {
        pivot.setPosition(0);
        swingyBoi.setPosition(0.482);
        juul.setPosition(0.2);
    }
    private void home3() {
        pivot.setPosition(0);
        swingyBoi.setPosition(0.482);
        juul.setPosition(0);
    }


    public void lower() {


        home1();

        opMode.sleep(400);


        pivot.setPosition(0.216);
        swingyBoi.setPosition(0.07);
        juul.setPosition(0.53);
        opMode.sleep(400);
        pivot.setPosition(0.516);


    }

    public void hitRed() {
        lower();

        opMode.sleep(1000);

        switch(getColor()) {
            case Color.RED:
                left();
                break;
            case Color.BLUE:
                right();
                break;
            default:
                opMode.telemetry.addData("Not Found", null);
                opMode.telemetry.update();
                opMode.sleep(1000);
                break;
        }
        opMode.sleep(700);
        home1();
        opMode.sleep(700);
        home2();
        opMode.sleep(700);
        home3();
    }

    public void hitBlue() {
        lower();

        opMode.sleep(1000);

        switch(getColor()) {
            case Color.RED:
                right();
                break;
            case Color.BLUE:
                left();
                break;
            default:
                opMode.telemetry.addData("Not Found", null);
                opMode.telemetry.update();
                opMode.sleep(1000);
                break;
        }
        opMode.sleep(700);
        home1();
        opMode.sleep(700);
        home2();
        opMode.sleep(700);
        home3();
    }

    public void addTelemetry() {
        opMode.telemetry.addData("juul servo", juul.getPosition());
        opMode.telemetry.addData("swing servo", swingyBoi.getPosition());
        opMode.telemetry.addData("color blue", colorSensor.blue());
        opMode.telemetry.addData("color red", colorSensor.red());
        opMode.telemetry.addData("color green", colorSensor.green());
    }
}
