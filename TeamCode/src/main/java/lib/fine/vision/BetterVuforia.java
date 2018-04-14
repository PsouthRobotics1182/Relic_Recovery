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
import org.opencv.android.Utils;
import org.opencv.calib3d.StereoBM;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.InputStreamReader;
import java.io.Reader;
import java.util.List;

import boofcv.abst.feature.disparity.StereoDisparity;
import boofcv.alg.filter.derivative.LaplacianEdge;
import boofcv.android.ConvertBitmap;
import boofcv.android.VisualizeImageData;
import boofcv.factory.feature.disparity.DisparityAlgorithms;
import boofcv.factory.feature.disparity.FactoryStereoDisparity;
import boofcv.io.calibration.CalibrationIO;
import boofcv.struct.calib.CameraPinholeRadial;
import boofcv.struct.geo.AssociatedPair;
import boofcv.struct.image.GrayF32;
import boofcv.struct.image.GrayS16;
import boofcv.struct.image.GrayU8;
import boofcv.struct.image.ImageBase;

/**
 * Created by drew on 11/25/17.
 */

@Autonomous
public class BetterVuforia {
    private VuforiaLocalizer vuforia;
    private VuforiaTrackables keys;
    private VuforiaTrackable cryptoKey;
    private LinearOpMode opMode;

    static {
        System.loadLibrary("opencv_java3");
    }

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

    public ImageView getPreview() {
        int previewID = opMode.hardwareMap.appContext.getResources().getIdentifier("boofPreview", "id", opMode.hardwareMap.appContext.getPackageName());
        return (ImageView) ((Activity) opMode.hardwareMap.appContext).findViewById(previewID);
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


    public void displayBitmap(final Bitmap bMap) {
        final ImageView imageView = getPreview();
        imageView.post(new Runnable() {
            public void run() {
                imageView.setImageBitmap(bMap);
            }
        });
    }

    public void displayBoof(final ImageBase boof) {
        Bitmap bMap = Bitmap.createBitmap(boof.getWidth(), boof.getHeight(), Bitmap.Config.ARGB_8888);
        ConvertBitmap.boofToBitmap(boof, bMap, null);

        displayBitmap(bMap);
    }

    public void displayCurrentFrame() throws InterruptedException {
        displayBitmap(getFrameAsBitmap());
    }

    public GrayU8 getFrameAsBoof() throws InterruptedException {
        Bitmap bm = getFrameAsBitmap();
        GrayU8 imag = ConvertBitmap.bitmapToGray(bm, null, GrayU8.class, null);
        return imag;
    }

    public Mat getFramAsMat() throws InterruptedException{
        Bitmap bm = getFrameAsBitmap();
        Mat right = new Mat();
        Utils.bitmapToMat(bm, right);
        return right;
    }

    public Bitmap getFrameAsBitmap() throws InterruptedException {
        VuforiaLocalizer.CloseableFrame frame = vuforia.getFrameQueue().take(); //takes the frame at the head of the queue
        Image rgb = null;

        long numImages = frame.getNumImages();

        for (int i = 0; i < numImages; i++) {
            if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                rgb = frame.getImage(i);
                break;
            }
        }
        Bitmap bm = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
        bm.copyPixelsFromBuffer(rgb.getPixels());

        Matrix matrix = new Matrix();
        matrix.postRotate(90);
        bm =  Bitmap.createBitmap(bm, 0, 0, bm.getWidth(), bm.getHeight(), matrix, true);

        return bm;
    }

    public static CameraPinholeRadial loadIntrinsic(LinearOpMode opMode) {
        CameraPinholeRadial intrinsic = null;
        //String user = System.getProperty("user.dir");
        //Log.e("DemooMain", System.getProperty("user.dir"));


        File mediaStorageDir = new File(Environment.getExternalStorageDirectory()
                + "/Android/data/"
                + "FIRST"
                + "/boofImages/config.txt");


        try {
            FileInputStream fos = new FileInputStream(mediaStorageDir);
            Reader reader = new InputStreamReader(fos);
            intrinsic = CalibrationIO.load(reader);
            opMode.telemetry.addData("Got?", "yes");
        } catch (RuntimeException e) {
            Log.w("DemoMain", "Failed to load intrinsic parameters: "+e.getClass().getSimpleName());
            e.printStackTrace();
            opMode.telemetry.addData("Got?", "no");

        } catch (FileNotFoundException ignore) {
            opMode.telemetry.addData("Got?", "no");

        }
        opMode.telemetry.update();

        return intrinsic;//new CameraPinholeRadial();
    }

    private static final int minDisparity = 15;
    private static final int maxDisparity = 255;

    public Bitmap stereoProcessAsBoof(GrayU8 distortedLeft, GrayU8 distortedRight) {
        List<AssociatedPair> matchedFeatures = FinedementalMatrii.computeMatches(distortedLeft, distortedRight);

        // compute disparity
        StereoDisparity<GrayS16, GrayF32> disparityAlg =
                FactoryStereoDisparity.regionSubpixelWta(DisparityAlgorithms.RECT,
                        minDisparity, maxDisparity, 5, 5, 20, 1, 0.1, GrayS16.class);


        GrayS16 derivLeft = new GrayS16(distortedLeft.width,distortedLeft.height);
        GrayS16 derivRight = new GrayS16(distortedRight.width,distortedRight.height);
        LaplacianEdge.process(distortedLeft, derivLeft);
        LaplacianEdge.process(distortedRight,derivRight);

        disparityAlg.process(derivLeft, derivRight);
        GrayF32 disparity = disparityAlg.getDisparity();

        // show results
        Bitmap bm = Bitmap.createBitmap(disparity.getWidth(), disparity.getHeight(), Bitmap.Config.ARGB_8888);
        VisualizeImageData.disparity(disparity, minDisparity, maxDisparity, 0, bm, null);
        return bm;
    }

    public Bitmap stereoProcessAsCV (Mat left, Mat right) {

        StereoBM stereo = StereoBM.create(16, 15);

        Mat disparity = new Mat();
        stereo.compute(left, right, disparity);

        Mat disP = new Mat(disparity.rows(), disparity.cols(), CvType.CV_8UC1);
        disparity.convertTo(disP, CvType.CV_8UC1);

        Bitmap disBMap = Bitmap.createBitmap(disparity.width(), disparity.height(), Bitmap.Config.ARGB_8888);
        Utils.matToBitmap(disP, disBMap);
        return disBMap;
    }


}
