package lib.fine.vision;

import android.app.Activity;
import android.graphics.Bitmap;
import android.graphics.Matrix;
import android.os.Environment;
import android.util.Log;
import android.widget.ImageView;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.CameraDevice;
import com.vuforia.HINT;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.InputStreamReader;
import java.io.Reader;
import java.util.List;


/**
 * Created by drew on 11/25/17.
 */

@Autonomous
public class BetterVuforia {
    private VuforiaLocalizer vuforia;
    private VuforiaTrackables keys;
    private VuforiaTrackable cryptoKey;
    private LinearOpMode opMode;


    public BetterVuforia(LinearOpMode opMode, boolean display) {
        this.opMode = opMode;
        VuforiaLocalizer.Parameters parameters;
        if (display) {
            int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
            parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        } else {
            parameters = new VuforiaLocalizer.Parameters();
        }

        parameters.vuforiaLicenseKey = "AUgSTBn/////AAAAGSU/cD15/UsujI6xLYV74ziGgnCxnhNN3o+oqCbjOAYeuTL3onL+U3IeZxlEpkmbZUZo3dM9ASoSZmIJSdJD4qql7aQoGkyiMmQrG0VrtDRYXGfD0S2gkiP9zyr+Cq+j0OFfrefZrq+k+29VF6ON1KOoPJdDVfUvfbj96xmLd9E6p3bGoJUQSbgnGu+ZkMK2+0Qu8tFe6v8Wx+0v3amf6kgOAaLbjdGqAygEwk9pEOWFxIjpUcwZj8qNqZvtRJP+7csocK3MYC+stHvVh42xXaXeShzC737bkSj0G4lWCtI3JNFDw6NRKX0dmwLbIVMizvudFRXwF2SahUpwh+h/2T5WWSfWP3lcrDYQRgJ54PWG";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 1);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        vuforia.setFrameQueueCapacity(1);

        CameraDevice.getInstance().setFocusMode(CameraDevice.FOCUS_MODE.FOCUS_MODE_CONTINUOUSAUTO);
        CameraDevice.getInstance().setFlashTorchMode(true);

        keys = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        cryptoKey = keys.get(0);
        cryptoKey.setName("column"); // can help in debugging; otherwise not necessary
    }


    public void activate() {
        keys.activate();
    }

    public void deactiviate() {
        keys.deactivate();
    }
    public RelicRecoveryVuMark getColumn() {
        return RelicRecoveryVuMark.from(cryptoKey);
    }


}
