package org.firstinspires.ftc.teamcode.secondrobot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.photo.Photo;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;

public class DetectBlockActions {
    OpenCvWebcam camera;
    Point center = new Point(0,0);
    double angle = 0;
    boolean isRed = false;

    public DetectBlockActions(HardwareMap hardwareMap) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        camera.setPipeline(new ContoursPipeline());

        camera.setMillisecondsPermissionTimeout(5000); // Timeout for obtaining permission is configurable. Set before opening.
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        //Image MUST be png, to have same type as input
        //Use UtilityFrameCapture to grab frame, USB-C to robot, grab VisionPortal-CameraFrameCapture- latest one
        //Crop down to size

    }

    public void setIsRed(boolean isRed) {
        this.isRed = isRed;
    }

    public Point pixelToPosition() {
        //TODO
        Point position = center;
        return position;
    }

    public Mat getColorThreshold(Mat hsv) {
        if (isRed) {
            Mat mask1 = new Mat();
            Mat mask2 = new Mat();
            Core.inRange(hsv, new Scalar(0, 70, 50), new Scalar(10, 255, 255), mask1);
            Core.inRange(hsv, new Scalar(170, 70, 50), new Scalar(180, 255, 255), mask2);
            Mat mask = new Mat();
            Core.bitwise_or(mask1, mask2, mask);
            return mask;
        } else {
            Mat mask = new Mat();
            Core.inRange(hsv, new Scalar(100, 70, 0), new Scalar(140, 255, 255), mask);
            return mask;
        }
    }
    public Mat getYellowThreshold(Mat hsv) {
        Mat mask = new Mat();
        Core.inRange(hsv, new Scalar(15, 70, 0), new Scalar(30, 255, 255), mask);
        return mask;
    }
    public Mat prepareForContours(Mat input) {
        Mat img = input;
        img = img.submat(0, img.rows(), 0, img.cols());

        Mat hsv = new Mat();
        Imgproc.cvtColor(img, hsv, Imgproc.COLOR_BGR2HSV);

        Mat colorMask = getColorThreshold(hsv);
        Mat yellowMask = getYellowThreshold(hsv);
        Mat mask = new Mat();
        Core.bitwise_or(colorMask, yellowMask, mask);

        Imgproc.blur(mask, mask, new Size(mask.width() / 40, mask.width() / 40));

        Mat secondRoundMasking = new Mat();
        Core.inRange(mask, new Scalar(254, 254, 254), new Scalar(255, 255, 255), secondRoundMasking);


        Mat canny = new Mat();
        Imgproc.Canny(secondRoundMasking, canny, 100, 200);

        return canny;
    }
    public void contourAndOval(Mat canny) {
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(canny, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        double minDistance = 100000;
        int minContour = 0;
        RotatedRect minOval = new RotatedRect();
        double maxArea = 100000000;
        for (int i = 0; i < contours.size(); i++) {
            if (contours.get(i).rows() > 5) {
                RotatedRect newOval = Imgproc.fitEllipseAMS(contours.get(i));
                double distance = Math.sqrt(Math.pow(newOval.center.x - (canny.cols() / 2), 2) + Math.pow(newOval.center.y - canny.rows(), 2));
                double area = newOval.size.area();
                if (area < maxArea && distance < minDistance) {
                    minOval = newOval;
                    minDistance = distance;
                }
            }
        }

        if (minOval != new RotatedRect()) {
            Point center = minOval.center;
            double angle = minOval.angle;
            setCenterAndAngle(center, angle);
        }
    }
    public void setCenterAndAngle(Point center, double angle) {
        this.center = center;
        this.angle = angle;
    }
    public Point getCenter() {
        return center;
    }
    public double getAngle() {
        return angle;
    }
    class ContoursPipeline extends OpenCvPipeline {
        boolean viewportPaused;

        @Override
        public Mat processFrame(Mat input) {

           Mat processed = prepareForContours(input);
           contourAndOval(processed);



            /**
             * NOTE: to see how to get data from your pipeline to your OpMode as well as how
             * to change which stage of the pipeline is rendered to the viewport when it is
             * tapped, please see {@link PipelineStageSwitchingExample}
             */

            return input;
        }

        @Override
        public void onViewportTapped() {
            viewportPaused = !viewportPaused;

            if (viewportPaused) {
                camera.pauseViewport();
            } else {
                camera.resumeViewport();
            }
        }
    }
}
