package org.firstinspires.ftc.teamcode.secondrobot;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Path;
import android.util.Log;

import androidx.annotation.ColorInt;

import com.qualcomm.robotcore.util.SortOrder;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

class ContourLocatorProcessorImpl extends ContourLocatorProcessor implements VisionProcessor
{
    ContourLocatorProcessorImpl(){}

    Point center = new Point(0,0);
    double angle = 0;
//    boolean isRed = false;
    int pixelWidth = 640;
    int pixelHeight = 480;
    @Override
    public void init(int width, int height, CameraCalibration calibration) {}

    public Point pixelToPosition(Point center) { //inches
        double pixelX = center.y;
        double pixelY = -center.x * 1.05;
        double degreesPerPixel = 63.0 / (double) pixelWidth;
        double YOffsetDegrees = 53; //TODO
        double middleY = pixelWidth / 2.0;
        double degreesX = (pixelX - ((double) pixelHeight / 2.0)) * degreesPerPixel;
        double degreesY = (pixelY) * degreesPerPixel + YOffsetDegrees;
        double cameraHeight = 2.22; //Inches, from top of block to camera lens, TODO
        double distanceFromCameraBase = cameraHeight * Math.tan(Math.toRadians(90 - degreesY));
        double x = distanceFromCameraBase * Math.cos(Math.toRadians(90.0 - degreesX));
        double y = distanceFromCameraBase * Math.sin(Math.toRadians(90.0 - degreesX));
        Point position = new Point(x, y - 1.8164);
        return position;
    }

    public Mat getColorThreshold(Mat hsv) {
        Mat blue = new Mat();
        Core.inRange(hsv, new Scalar(0, 30, 160), new Scalar(110, 205, 245), blue);
//        if (isRed) {
//            Mat mask1 = new Mat();
//            Mat mask2 = new Mat();
//            Core.inRange(hsv, new Scalar(0, 10, 190), new Scalar(100, 255, 255), mask1);
//            Core.inRange(hsv, new Scalar(170, 10, 200), new Scalar(255, 135, 255), mask2);
//            Mat combinedMask = new Mat();
//            Core.bitwise_or(mask1, mask2, combinedMask);
//            Mat notBlue = new Mat();
//            Core.inRange(blue, new Scalar(0,0,0), new Scalar(1,1,1), notBlue);
//            Mat finalMask = new Mat();
//            Core.bitwise_and(combinedMask, notBlue, finalMask);
//            Imgcodecs.imwrite("src/main/java/org/firstinspires/ftc/teamcode/testing/tests/data/red.jpg", finalMask);
//            return finalMask;
//        } else {
            Imgcodecs.imwrite("src/main/java/org/firstinspires/ftc/teamcode/testing/tests/data/blue.jpg", blue);
            return blue;
//        }
    }
    public Mat getYellowThreshold(Mat hsv) {
        Mat mask = new Mat();
        Core.inRange(hsv, new Scalar(10, 200, 160), new Scalar(45, 245, 255), mask);
        Imgcodecs.imwrite("src/main/java/org/firstinspires/ftc/teamcode/testing/tests/data/yellow.jpg", mask);
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

//        Imgproc.blur(mask, mask, new Size(mask.width() / 80, mask.width() / 80));

//        Mat secondRoundMasking = new Mat();
//        Core.inRange(mask, new Scalar(254, 254, 254), new Scalar(255, 255, 255), secondRoundMasking);

        Mat element = Imgproc.getStructuringElement(Imgproc.CV_SHAPE_ELLIPSE, new Size(mask.width() / 35, mask.width() / 35));
        Imgproc.dilate(mask, mask, element);


        Mat canny = new Mat();
        Imgproc.Canny(mask, canny, 100, 200);

        Imgcodecs.imwrite("src/main/java/org/firstinspires/ftc/teamcode/testing/tests/data/final mask edges.jpg", canny);
        return canny;
    }

    public List<MatOfPoint> contours = new ArrayList<>();
    public List<MatOfPoint> getContours() {
        return contours;
    }
    RotatedRect minOval;
    public RotatedRect getMinOval() {
        return minOval;
    }
    public void contourAndOval(Mat canny) {
        Mat contoursMat = canny;
        contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(canny, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        double minDistance = 100000;
        int minContour = 0;
        RotatedRect minOval = new RotatedRect();
        double maxArea = 100000;
        double minArea = 2000;
        double maxHeight = 220;
        for (int i = 0; i < contours.size(); i++) {
            if (contours.get(i).rows() > 5) {
                RotatedRect newOval = Imgproc.fitEllipseDirect(contours.get(i));
                double distance = Math.sqrt(Math.pow(newOval.center.x, 2.0) + Math.pow(newOval.center.y - (pixelHeight / 2), 2.0));
                double area = newOval.size.area();
                Imgproc.ellipse(contoursMat, newOval, new Scalar(200, 200, 200));
                if (minArea < area && area < maxArea && distance < minDistance) {
                    minOval = newOval;
                    minDistance = distance;
                }
            }
        }
        Imgcodecs.imwrite("src/main/java/org/firstinspires/ftc/teamcode/testing/tests/data/final mask contours.jpg", contoursMat);
        this.minOval = minOval;

        if (minOval != new RotatedRect()) {
            Point center = new Point(minOval.center.x, minOval.center.y + minOval.boundingRect().height / 2);
            double angle = minOval.angle;
            setCenterAndAngle(center, angle);
//            RobotLog.dd("Camera", "center %f, %f", center.x, center.y);
        } else {
            Point center = new Point(pixelWidth / 2.0, pixelHeight / 2.0);
            setCenterAndAngle(center, 0);
        }
        return;
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

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos)
    {
        Imgcodecs.imwrite("/sdcard/FIRST/java/src/img.jpg", frame);
        Mat processed = prepareForContours(frame);
        Imgcodecs.imwrite("/sdcard/FIRST/java/src/processed.jpg", processed);
        contourAndOval(processed);

        /**
         * NOTE: to see how to get data from your pipeline to your OpMode as well as how
         * to change which stage of the pipeline is rendered to the viewport when it is
         * tapped, please see {@link PipelineStageSwitchingExample}
         */

        return processed;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {}
}
