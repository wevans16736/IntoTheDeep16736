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
    boolean isRed = false;
    int pixelWidth = 640;
    int pixelHeight = 480;
    @Override
    public void init(int width, int height, CameraCalibration calibration) {}


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
            Mat mask1 = new Mat();
            Core.inRange(hsv, new Scalar(60, 50, 0), new Scalar(140, 255, 255), mask1);
            Mat floor = new Mat();
            Core.inRange(hsv, new Scalar(0, 50, 110), new Scalar(255, 60, 255), floor);
            Core.inRange(floor, new Scalar(0, 0, 0), new Scalar(1, 1, 1), floor);
            Mat mask = new Mat();
            Core.bitwise_and(mask1, floor, mask);
            Imgcodecs.imwrite("src/main/java/org/firstinspires/ftc/teamcode/testing/tests/data/blue.jpg", mask);
            return floor;
        }
    }
    public Mat getYellowThreshold(Mat hsv) {
        Mat mask = new Mat();
        Core.inRange(hsv, new Scalar(10, 70, 70), new Scalar(30, 255, 255), mask);
        Imgcodecs.imwrite("src/main/java/org/firstinspires/ftc/teamcode/testing/tests/data/yellow.jpg", mask);
        return mask;
    }
    public Mat prepareForContours(Mat input) {
        Mat img = input;
        img = img.submat(0, img.rows(), 0, img.cols());

        Mat hsv = new Mat();
        Imgproc.cvtColor(img, hsv, Imgproc.COLOR_RGB2HSV);

        Mat colorMask = getColorThreshold(hsv);
        Mat yellowMask = getYellowThreshold(hsv);
        Mat mask = new Mat();
        Core.bitwise_or(colorMask, yellowMask, mask);

        Imgproc.blur(mask, mask, new Size(mask.width() / 30, mask.width() / 30));

        Mat secondRoundMasking = new Mat();
        Core.inRange(mask, new Scalar(254, 254, 254), new Scalar(255, 255, 255), secondRoundMasking);

        Mat element = Imgproc.getStructuringElement(Imgproc.CV_SHAPE_RECT, new Size(mask.width() / 45, mask.width() / 45));
        Imgproc.dilate(secondRoundMasking, secondRoundMasking, element);


        Mat canny = new Mat();
        Imgproc.Canny(secondRoundMasking, canny, 100, 200);

        Imgcodecs.imwrite("src/main/java/org/firstinspires/ftc/teamcode/testing/tests/data/final mask edges.jpg", canny);
        return canny;
    }

    public List<MatOfPoint> contours = new ArrayList<>();
    public void contourAndOval(Mat canny) {
        contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(canny, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        double minDistance = 100000;
        int minContour = 0;
        RotatedRect minOval = new RotatedRect();
        double maxArea = 100000000; //TODO
        double minArea = 100000; //100,000
        for (int i = 0; i < contours.size(); i++) {
            if (contours.get(i).rows() > 5) {
                RotatedRect newOval = Imgproc.fitEllipseAMS(contours.get(i));
                double distance = Math.sqrt(Math.pow(newOval.center.x - (canny.cols() / 2.0), 2.0) + Math.pow(newOval.center.y - (canny.rows() / 2.0), 2.0));
                double area = newOval.size.area();
                if (minArea < area && area < maxArea && distance < minDistance) {
                    minOval = newOval;
                    minDistance = distance;
                }
            }
        }

        if (minOval != new RotatedRect()) {
            Point center = minOval.center;
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

    public void setIsRed(boolean isRed) {
        this.isRed = isRed;
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos)
    {

        Imgcodecs.imwrite("/sdcard/FIRST/java/src/img.png", frame);
        Mat processed = prepareForContours(frame);
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
