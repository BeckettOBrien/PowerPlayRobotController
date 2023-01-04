package org.firstinspires.ftc.teamcode.util.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public class SignalSleeveDetector {

    OpenCvCamera camera;
    SignalSleevePipeline pipeline;

    public SignalSleeveDetector(HardwareMap hw, double xOffset) {
        int cameraMonitorViewId = hw.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hw.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hw.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        camera.openCameraDevice();
        pipeline = new SignalSleevePipeline();
        camera.setPipeline(pipeline);
        stream();
    }

    public void stream() {
        camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
        FtcDashboard.getInstance().startCameraStream(camera, 30);
    }

    public void stop() {
        camera.stopStreaming();
        camera.closeCameraDeviceAsync(() -> {});
    }

}

@Config
class SignalSleevePipeline extends OpenCvPipeline {

    public static int COLOR_BOUND_LOW = 5;
    public static int COLOR_BOUND_HIGH = 20;

    double roix1 = 500;
    double roiy1 = 200;
    double roix2 = 800;
    double roiy2 = 500;

    Scalar lowPurple = new Scalar(150, 85, 125);
    Scalar highPurple = new Scalar(175, 255, 255);
    Scalar lowGreen = new Scalar(COLOR_BOUND_LOW, 0, 100);
    Scalar highGreen = new Scalar(COLOR_BOUND_HIGH, 255, 255);
    Scalar lowOrange = new Scalar(COLOR_BOUND_LOW, 0, 100);
    Scalar highOrange = new Scalar(COLOR_BOUND_HIGH, 255, 255);

    Mat mat = new Mat();
    Rect ROI;

    public SignalSleevePipeline() {
        ROI = new Rect(
                new Point(roix1, roiy1),
                new Point(roix2, roiy2)
        );
    }

    public static boolean showGrayscale = false;

    @Override
    public Mat processFrame(Mat input) {

        Mat out = new Mat();

        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        Mat purple = mat.submat(ROI);
        Core.inRange(mat, lowPurple, highPurple, purple);
        double purpleValue = Core.mean(purple).val[0];
        purple.copyTo(out);
        purple.release();

        Mat green = mat.submat(ROI);
        Core.inRange(mat, lowGreen, highGreen, green);
        double greenValue = Core.mean(green).val[0];
        green.release();

        Mat orange = mat.submat(ROI);
        Core.inRange(mat, lowOrange, highOrange, orange);
        double orangeValue = Core.mean(orange).val[0];
        orange.release();

//        input.copyTo(out);

        // Draw ROI rectangles
        Scalar rectColor = new Scalar(255, 255, 255);
        double max = Math.max(Math.max(purpleValue, greenValue), orangeValue);
        if (max == purpleValue) {
            rectColor = new Scalar(255, 0, 255);
        } else if (max == greenValue) {
            rectColor = new Scalar(0, 255, 0);
        } else if (max == orangeValue) {
            rectColor = new Scalar(255, 150, 0);
        }
//        Scalar foundColor = new Scalar(0, 255, 0);
        Imgproc.rectangle(out, ROI, rectColor);
//        Imgproc.rectangle(mat, LEFT_ROI, (currentLoc == OpenCVElementTracker.LOCATION.LEFT) ? foundColor : emptyColor);
//        Imgproc.rectangle(mat, RIGHT_ROI, (currentLoc == OpenCVElementTracker.LOCATION.RIGHT) ? foundColor : emptyColor);

        return out;
    }

    @Override
    public void onViewportTapped() {
        super.onViewportTapped();
        showGrayscale = !showGrayscale;
    }

//    public OpenCVElementTracker.LOCATION getLocation() {
//        return currentLoc;
//    }

}