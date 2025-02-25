package org.firstinspires.ftc.teamcode.testing.tests;

import static org.opencv.core.CvType.CV_8UC1;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.secondrobot.ContourLocatorProcessor;
import org.firstinspires.ftc.teamcode.secondrobot.DetectBlockActions;
import org.junit.Assert;
import org.junit.Test;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDMatch;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.features2d.ORB;
import org.opencv.features2d.BFMatcher;
import org.opencv.features2d.Features2d;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
//import java.awt.image.BufferedImage;

public class ImageProcessing {

    @Test
    public void testRedButter(){
//        OpenCVLoader.initDebug();

        int i = CvType.CV_16UC4;
        String pathImg = "src/main/java/org/firstinspires/ftc/teamcode/testing/tests/data/redButter1.jpg";

        File fileImg = new File(pathImg);
        String absolutePathImg = fileImg.getAbsolutePath();


//        try {
//            Image img = ImageIO.read(new File("strawberry.jpg"));
//        } catch (IOException e) {
//        }
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
        /* In order for this to work, download OpenCV version 4.7.0 and extract the file.
        In the OpenCV file, go into build then java then x64.
        Copy the opencv_java470.dll and paste it into Program Files/Android/AndroidStudio/jre/bin
        */

        Mat img = Imgcodecs.imread(absolutePathImg, Imgcodecs.IMREAD_COLOR);
        img = img.submat(0, img.rows(), 0, img.cols());
        Mat imgNew = img.submat(370, 660, 770, 1260);
        int blockHeight = 190;
        int blockLength = 450;
        int blockPos[] = new int[0];
        for (int j = 0; j < img.rows() - blockHeight; j = j + 10) {
            for (int k = 0; k < img.cols() - blockLength; k = k + 10) {
                imgNew = img.submat(j, j + blockHeight, k, k + blockLength);
                Scalar mean = Core.mean(imgNew);
                if (mean.val[2] > (mean.val[0] + mean.val[1]) * 1.2) {
                    int newBlockPos[] = new int[blockPos.length + 2];
                    for (int l = 0; l < blockPos.length; l++) {
                        newBlockPos[l] = blockPos[l];
                    }
                    newBlockPos[blockPos.length] = k;
                    newBlockPos[blockPos.length + 1] = j;
                    blockPos = newBlockPos;
                }
            }
        }

        imgNew = img.submat(400, 400+blockHeight, 930, 930+blockLength);
        Imgcodecs.imwrite("src/main/java/org/firstinspires/ftc/teamcode/testing/tests/data/imgNew.jpg", imgNew);
        Assert.assertEquals(1000, Core.mean(imgNew).val[2], 0);

//        img = loadImage("imgR");
//        Point resultR = openCv.templateMatching(img, templ);
//        img = loadImage("imgM");
//        Point resultM = openCv.templateMatching(img, templ);
//        Assert.assertEquals(250, resultL.y, 10);
    }
    @Test
    public void testBlueButter(){
//        OpenCVLoader.initDebug();

        int i = CvType.CV_16UC4;
        String pathImg = "src/main/java/org/firstinspires/ftc/teamcode/testing/tests/data/blueButter0.jpg";

        File fileImg = new File(pathImg);
        String absolutePathImg = fileImg.getAbsolutePath();


//        try {
//            Image img = ImageIO.read(new File("strawberry.jpg"));
//        } catch (IOException e) {
//        }
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
        /* In order for this to work, download OpenCV version 4.7.0 and extract the file.
        In the OpenCV file, go into build then java then x64.
        Copy the opencv_java470.dll and paste it into Program Files/Android/AndroidStudio/jre/bin
        */

        Mat img = Imgcodecs.imread(absolutePathImg, Imgcodecs.IMREAD_COLOR);
        img = img.submat(0, img.rows(), 0, img.cols());
        Mat imgNew = img.submat(370, 660, 770, 1260);
        int blockHeight = 190;
        int blockLength = 450;
        int blockPos[] = new int[0];
        for (int j = 0; j < img.rows() - blockHeight; j = j + 10) {
            for (int k = 0; k < img.cols() - blockLength; k = k + 10) {
                imgNew = img.submat(j, j + blockHeight, k, k + blockLength);
                Scalar mean = Core.mean(imgNew);
                if (mean.val[2] > (mean.val[0] + mean.val[1]) * 1.2) {
                    int newBlockPos[] = new int[blockPos.length + 2];
                    for (int l = 0; l < blockPos.length; l++) {
                        newBlockPos[l] = blockPos[l];
                    }
                    newBlockPos[blockPos.length] = k;
                    newBlockPos[blockPos.length + 1] = j;
                    blockPos = newBlockPos;
                }
            }
        }

        imgNew = img.submat(400, 400+blockHeight, 930, 930+blockLength);
        Imgcodecs.imwrite("src/main/java/org/firstinspires/ftc/teamcode/testing/tests/data/imgNew.jpg", imgNew);
        Assert.assertEquals(1000, Core.mean(imgNew).val[2], 0);

//        img = loadImage("imgR");
//        Point resultR = openCv.templateMatching(img, templ);
//        img = loadImage("imgM");
//        Point resultM = openCv.templateMatching(img, templ);
//        Assert.assertEquals(250, resultL.y, 10);
    }
    @Test
    public void testFeatureMapping(){

        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

        String pathImg = "src/main/java/org/firstinspires/ftc/teamcode/testing/tests/data/redButter1.jpg";

        File fileImg = new File(pathImg);
        String absolutePathImg = fileImg.getAbsolutePath();

        Mat img = Imgcodecs.imread(absolutePathImg, Imgcodecs.IMREAD_COLOR);
        img = img.submat(0, img.rows(), 0, img.cols());

        String pathTempl = "src/main/java/org/firstinspires/ftc/teamcode/testing/tests/data/redButter_Template.jpg";

        File fileTempl = new File(pathTempl);
        String absolutePathTempl = fileTempl.getAbsolutePath();

        Mat templ = Imgcodecs.imread(absolutePathTempl, Imgcodecs.IMREAD_COLOR);
        templ = templ.submat(0, templ.rows(), 0, templ.cols());

        ORB orb = ORB.create();

        MatOfKeyPoint kpTempl = new MatOfKeyPoint();
        Mat desTempl = new Mat();
        orb.detectAndCompute(templ, new Mat(), kpTempl, desTempl);

        MatOfKeyPoint kpImg = new MatOfKeyPoint();
        Mat desImg = new Mat();
        orb.detectAndCompute(img, new Mat(), kpImg, desImg);

        BFMatcher bf = BFMatcher.create(Core.NORM_HAMMING, true);

        MatOfDMatch match = new MatOfDMatch();
        bf.match(desTempl, desImg, match);
//        match = Collections.sort(match);
        Mat output = new Mat();
        Features2d.drawMatches(templ, kpTempl, img, kpImg, match, output, Features2d.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS);
        Imgcodecs.imwrite("src/main/java/org/firstinspires/ftc/teamcode/testing/tests/data/ORB.jpg", output);

        return;
    }
    public Mat testImgThresholding(){
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

        String pathImg = "src/main/java/org/firstinspires/ftc/teamcode/testing/tests/data/blockfest3.jpg";

        File fileImg = new File(pathImg);
        String absolutePathImg = fileImg.getAbsolutePath();

        Mat img = Imgcodecs.imread(absolutePathImg, Imgcodecs.IMREAD_COLOR);
        img = img.submat(0, img.rows(), 0, img.cols());

        Mat hsv = new Mat();
        Imgproc.cvtColor(img, hsv, Imgproc.COLOR_BGR2HSV);

        Mat mask1 = new Mat();
        Mat mask2 = new Mat();
        Core.inRange(hsv, new Scalar(0, 70, 50), new Scalar(10, 255, 255), mask1);
        Core.inRange(hsv, new Scalar(170, 70, 50), new Scalar(180, 255, 255), mask2);
        Mat mask = new Mat();
        Core.bitwise_or(mask1, mask2, mask);
//        Mat mask = new Mat();
//        Core.inRange(hsv, new Scalar(100, 70, 0), new Scalar(140, 255, 255), mask);
        Imgcodecs.imwrite("src/main/java/org/firstinspires/ftc/teamcode/testing/tests/data/final mask.jpg", mask);

//        Photo.fastNlMeansDenoising(mask, mask, 50000);

        Imgproc.blur(mask, mask, new Size(mask.width() / 40, mask.width() / 40));

        Mat secondRoundMasking = new Mat();
        Core.inRange(mask, new Scalar(254, 254, 254), new Scalar(255, 255, 255), secondRoundMasking);

        Imgcodecs.imwrite("src/main/java/org/firstinspires/ftc/teamcode/testing/tests/data/final mask denoised.jpg", secondRoundMasking);

        Mat canny = new Mat();
        Imgproc.Canny(secondRoundMasking, canny, 100, 200);

        Imgcodecs.imwrite("src/main/java/org/firstinspires/ftc/teamcode/testing/tests/data/final mask edges.jpg", canny);
        return canny;
    }
    @Test
    public void testRayThing() {

        Mat canny = testImgThresholding();
        Mat nonZeroPos = new Mat();
        Core.findNonZero(canny, nonZeroPos);
        double centerX = canny.cols() / 2;
        double centerY = canny.rows();
        double minDistance = 10000000;
        double minY = 0;
        double minX = 0;
        for (int i = 0; i < nonZeroPos.rows(); i++) {
            double positions[] = nonZeroPos.get(i, 0);
            double distance = Math.sqrt(Math.pow(positions[0] - centerY, 2) + Math.pow(positions[1] - centerX, 2));
            if (distance < minDistance) {
                minDistance = distance;
                minX = positions[0];
                minY = positions[1];
            }
        }

        double posOnRay[] = new double[0];
        for (int i = 0; i < nonZeroPos.rows(); i++) {
            double position[] = nonZeroPos.get(i, 0);
            if (position[0] == minX && position[1] != minY) {
                double newPos[] = new double[posOnRay.length + 2];
                for (int j = 0; j < posOnRay.length; j++) {
                    newPos[j+2] = posOnRay[j];
                }
                posOnRay = newPos;
                posOnRay[0] = position[0];
                posOnRay[1] = position[1];
            }
        }
        double closestOnRay[] = new double[2];

        closestOnRay[0] = posOnRay[0];
        closestOnRay[1] = posOnRay[1];
        for (int i = 0; i < posOnRay.length; i = i + 2) {
            if (posOnRay[i + 1] < closestOnRay[1]) {
                closestOnRay[0] = posOnRay[i];
                closestOnRay[1] = posOnRay[i+1];
            }
        }
        return;
    }
    @Test
    public void testContours(){
        Mat canny = testImgThresholding();
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(canny, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        double minDistance = 100000;
        int minContour = 0;
        for (int i = 0; i < contours.size(); i++) {
            double newPos[] = contours.get(i).get(0, 0);
            double distance = Math.sqrt(Math.pow(newPos[0] - canny.cols() / 2, 2) + Math.pow(newPos[1] - canny.rows(), 2));
            if (distance < minDistance) {
                minContour = i;
                minDistance = distance;
            }
        }
        Point center = Imgproc.fitEllipseAMS(contours.get(minContour)).center;
        double angle = Imgproc.fitEllipseAMS(contours.get(minContour)).angle;
        center = center;
        angle = angle;
    }
    @Test
    public void testDetectBlockActions() {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

        String pathImg = "src/main/java/org/firstinspires/ftc/teamcode/testing/tests/data/blueButter3.jpg";

        File fileImg = new File(pathImg);
        String absolutePathImg = fileImg.getAbsolutePath();

        Mat img = Imgcodecs.imread(absolutePathImg, Imgcodecs.IMREAD_COLOR);
//        DetectBlockActions detectBlockActions = new DetectBlockActions();
//        detectBlockActions.contourAndOval(detectBlockActions.prepareForContours(img));
//
//        double angle = detectBlockActions.getAngle();
//        Point center = detectBlockActions.getCenter();
//        List<MatOfPoint> contours = detectBlockActions.contours;
//        double contourArea[] = new double[contours.size()];
//        for (int i = 0; i < contourArea.length; i++) {
//            RotatedRect oval = Imgproc.fitEllipseAMS(contours.get(i));
//            contourArea[i] = oval.size.area();
//        }
    }
    @Test
    public void testContourLocatorProcessor() {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

        String pathImg = "src/main/java/org/firstinspires/ftc/teamcode/testing/tests/data/blockfest1.jpg";

        File fileImg = new File(pathImg);
        String absolutePathImg = fileImg.getAbsolutePath();
        Mat img = Imgcodecs.imread(absolutePathImg, Imgcodecs.IMREAD_COLOR);

        Imgproc.cvtColor(img, img, Imgproc.COLOR_BGR2RGB);
//        Imgcodecs.imwrite("src/main/java/org/firstinspires/ftc/teamcode/testing/tests/data/bgr.jpg", img);
//        Imgproc.cvtColor(img, img, Imgproc.COLOR_BGR2HSV);
//        Imgcodecs.imwrite("src/main/java/org/firstinspires/ftc/teamcode/testing/tests/data/hsv.jpg", img);

        int type = img.type();
        ContourLocatorProcessor contourLocatorProcessor = new ContourLocatorProcessor.Builder()
                .build();
        contourLocatorProcessor.contourAndOval(contourLocatorProcessor.newMethod(img));
        List<MatOfPoint> contours = contourLocatorProcessor.getContours();
        double contourArea[] = new double[contours.size()];
        double contourX[] = new double[contours.size()];
        double contourY[] = new double[contours.size()];

        for (int i = 0; i < contourArea.length; i++) {
            if (contours.get(i).rows() > 5) {
                RotatedRect oval = Imgproc.fitEllipseDirect(contours.get(i));
                contourArea[i] = oval.size.area();
                contourX[i] = oval.center.x;
                contourY[i] = oval.center.y;
//                distance[i] = Math.sqrt(Math.pow(oval.center.x - (640 / 2.0), 2.0) + Math.pow(oval.center.y - (pixelHeight), 2.0));

            }
        }
        Point center = contourLocatorProcessor.getCenter();
        Point center1 = contourLocatorProcessor.pixelToPosition(contourLocatorProcessor.getCenter());
    }
    @Test
    public void testNewMethod() {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

        String pathImg = "src/main/java/org/firstinspires/ftc/teamcode/testing/tests/data/blockfest1.jpg";

        File fileImg = new File(pathImg);
        String absolutePathImg = fileImg.getAbsolutePath();
        Mat img = Imgcodecs.imread(absolutePathImg, Imgcodecs.IMREAD_COLOR);

//        Imgproc.cvtColor(img, img, Imgproc.COLOR_RGB2BGR);
        Imgproc.cvtColor(img, img, Imgproc.COLOR_BGR2RGB);
        Imgcodecs.imwrite("src/main/java/org/firstinspires/ftc/teamcode/testing/tests/data/rgb.jpg", img);
//        Imgproc.cvtColor(img, img, Imgproc.COLOR_BGR2HSV);
//        Imgproc.cvtColor(img, img, Imgproc.COLOR_BGR2GRAY);
        ContourLocatorProcessor contourLocatorProcessor = new ContourLocatorProcessor.Builder()
                .build();
        List<RotatedRect> ovals;
        ovals = contourLocatorProcessor.newContours(contourLocatorProcessor.newMethod(img));
        Imgproc.ellipse(img, contourLocatorProcessor.bestContour(ovals), new Scalar(0,0,0));
        double minDistance = 1000000;
        double maxDistance = 2.2;
        RotatedRect bestContour = new RotatedRect();
        for (int i = 0; i < ovals.size(); i++) {
            boolean isTooClose = false;
            for (int j = 0; j < ovals.size() - 1; j++) {
                int k = j;
                if (j >= i){
                    k++;
                }
                Point srcPnt = contourLocatorProcessor.pixelToPosition(ovals.get(i).center);
                Point dstPnt = contourLocatorProcessor.pixelToPosition(ovals.get(k).center);
                double distFromBlock = Math.sqrt(Math.pow(srcPnt.x - dstPnt.x, 2) + Math.pow(srcPnt.y - dstPnt.y, 2));
                if (distFromBlock < maxDistance) {
                    isTooClose = true;
                }
            }
            Point ovalCenter = contourLocatorProcessor.pixelToPosition(ovals.get(i).center);
            double distFromBase = Math.sqrt(Math.pow(Range.clip(ovalCenter.x, 0, 640), 2) + Math.pow(ovalCenter.y - (480.0 / 2.0), 2));
            if (!isTooClose && distFromBase < minDistance){
                bestContour = ovals.get(i);
                minDistance = distFromBase;
            }
        }
        Imgcodecs.imwrite("src/main/java/org/firstinspires/ftc/teamcode/testing/tests/data/final.jpg", img);

        double contourArea[] = new double[ovals.size()];
        double contourX[] = new double[ovals.size()];
        double contourY[] = new double[ovals.size()];
        for (int i = 0; i < ovals.size(); i++) {
                contourArea[i] = ovals.get(i).size.area();
                contourX[i] = ovals.get(i).center.x;
                contourY[i] = ovals.get(i).center.y;
        }
    }
    @Test
    public void testPixelToInches() {
//        DetectBlockActions detectBlockActions = new DetectBlockActions();
        Point center = new Point(400,240);
        double pixelWidth = 640;
        double pixelHeight = 480;
        double pixelX = center.y;
        double pixelY = -center.x * 1.05;
        double degreesPerPixel = 63.0 / (double) pixelWidth;
        double YOffsetDegrees = 53.4; //TODO
        double middleY = pixelWidth / 2.0;
        double degreesX = (pixelX - ((double) pixelHeight / 2.0)) * degreesPerPixel;
        double degreesY = (pixelY) * degreesPerPixel - YOffsetDegrees;
        double cameraHeight = 2.3; //Inches, from top of block to camera lens, TODO
        double distanceFromCameraBase = cameraHeight * Math.tan(Math.toRadians(90 - degreesY));
        double x = distanceFromCameraBase * Math.cos(Math.toRadians(90.0 - degreesX));
        double y = distanceFromCameraBase * Math.sin(Math.toRadians(90.0 - degreesX));
        Point position = new Point(x, y - 1.85666);
//        Point position = new Point(x, y);
//        detectBlockActions.setCenterTest(center);
//        center = detectBlockActions.pixelToPosition();


    }
}
