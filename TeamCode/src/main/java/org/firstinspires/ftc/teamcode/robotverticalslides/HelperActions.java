package org.firstinspires.ftc.teamcode.robotverticalslides;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robotverticalslides.HorizontalSlide.HorizontalIntakeActions;
import org.firstinspires.ftc.teamcode.robotverticalslides.HorizontalSlide.HorizontalSlideActions;
import org.firstinspires.ftc.teamcode.robotverticalslides.HorizontalSlide.HorizontalWristActions;
import org.firstinspires.ftc.teamcode.robotverticalslides.VerticalSlide.VerticalGrabberActions;
import org.firstinspires.ftc.teamcode.robotverticalslides.VerticalSlide.VerticalSlideActions;
import org.firstinspires.ftc.teamcode.robotverticalslides.VerticalSlide.VerticalWristActions;

public abstract class HelperActions extends LinearOpMode {
    public final double SPEED = 0.5;

    public static int LEFT = 1;
    public static int RIGHT = 2;
    public static int FORWARDS = 3;
    public static int BACKWARDS = 4;
    public static int LOW = 5;
    public static int MEDIUM = 6;
    public static int HIGH = 7;

    private int speeding = 0;
    private double speed = 0.6;

    /**drive speed configuration**/
    public double getSpeed() { return speed; }
    public void setSpeed(double speed) {
        this.speed = speed;
    }
    public void changeSpeed(DriveActions driveActions, boolean upOne, boolean downOne, boolean upTwo, boolean downTwo, double scalarMultiple) {
        if (upOne) {
            speeding++;
            if (speeding == 1) {
                speed = speed + 0.1;
            }
        }
        if (downOne) {
            speeding++;
            if (speeding == 1) {
                speed = speed - 0.1;
            }
        }
        if (upTwo) {
            speeding++;
            if (speeding == 1) {
                speed = speed + 0.2;
            }
        }
        if (downTwo) {
            speeding++;
            if (speeding == 1) {
                speed = speed - 0.2;
            }
        }
        if (!upOne && !downOne && !upTwo && !downTwo) {
            speeding = 0;
        }
        if (speed < 0) {
            speed = 0;
        }
        if (speed > 1.0) {
            speed = 1.0;
        }
        double speedLeft = 1.0 - speed;
        driveActions.setSpeed(speed + (scalarMultiple * speedLeft));
        telemetry.addData("speed: ", speed);
    }

    double prevSpeed;
    boolean low = false;
    boolean prevToggle = false;
    double lowSpeed = 0.35;
    public void toggleSpeed(boolean toggle) {
        if (toggle && !prevToggle) {
            low = !low;
            if (low) {
                prevSpeed = speed;
                speed = lowSpeed;
            } else {
                lowSpeed = speed;
                speed = prevSpeed;
            }
        }
         prevToggle = toggle;
    }

     /**slide configuration?**/
    boolean wasOverrideSlide = true;
    boolean overrideSlide = true;
    double overrideSlideThreshold = 325;
    public void updateExchangeAssembly(VerticalGrabberActions grabber, VerticalWristActions verticalWrist, HorizontalWristActions horizontalWrist, HorizontalSlideActions horizontalSlide, VerticalSlideActions verticalSlide) {
        //tells the vertical wrist when the grabber is closed
        verticalWrist.setGrabberClosed(grabber.isClose());
        //tells the vertical wrist when the slide is up
        verticalWrist.setSlideUp(verticalSlide.getSlidePosition() < -800);
        //tells the horizontal slide to stop and let the horizontal wrist flip up or flip down when going in or out
        overrideSlide = horizontalSlide.getSlidePosition() < overrideSlideThreshold && !horizontalWrist.override;
        horizontalWrist.setIsSlideIn(overrideSlide);
        overrideSlide(horizontalSlide);
        wasOverrideSlide = overrideSlide;
    }
    double startTime = 0;
    public void overrideSlide(HorizontalSlideActions horizontalSlide) {
        if (overrideSlide && !wasOverrideSlide) {
            startTime = System.currentTimeMillis();
        }
        if (System.currentTimeMillis() < startTime + 420) {
            horizontalSlide.setOverride(true);
        } else {
            horizontalSlide.setOverride(false);
        }
    }

    double verticalGrabberStartCloseTime = 0;
    double verticalGrabberStartOpenTime = 0;
    double verticalFlipBackStartTime = 0;
    double horizontalFlipBackStartTime = 0;
    boolean flipBack = false;
    public boolean close(VerticalGrabberActions verticalGrabber, VerticalWristActions verticalWrist, VerticalSlideActions verticalSlide, HorizontalWristActions horizontalWrist, HorizontalSlideActions horizontalSlide) {
        boolean close = true;
        double currentTime = System.currentTimeMillis();
        if (verticalWrist.forward) {
            if (!verticalGrabber.isClose()) {
                verticalGrabberStartCloseTime = currentTime;
                verticalGrabber.close();
            }
            if (currentTime > verticalGrabberStartCloseTime + 420) {
                verticalWrist.backward();
                verticalWrist.update();
                flipBack = true;
                verticalFlipBackStartTime = currentTime;
            }
            close = false;
            telemetry.addData("close", 1);
        }
        if (currentTime > verticalFlipBackStartTime + 420) {
            if (verticalGrabber.isClose()) {
                verticalGrabber.open();
                verticalGrabberStartOpenTime = currentTime;
                close = false;
                telemetry.addData("close", 2);
            }
        } else {
            close = false;
            telemetry.addData("close", 3);
        }
        if (currentTime < verticalGrabberStartOpenTime + 420) {
            close = false;
            telemetry.addData("close", 4);
        }
        if (verticalSlide.getSlidePosition() < -10) {
            verticalSlide.setSlidePosition(10, 2000);
            close = false;
            telemetry.addData("close", 5);
        }

        if (horizontalSlide.getSlidePosition() > 1) {
            if (horizontalWrist.forward && horizontalSlide.getSlidePosition() > overrideSlideThreshold) {
                horizontalFlipBackStartTime = currentTime;
                horizontalWrist.backward();
            }
            if (currentTime > horizontalFlipBackStartTime + 420) {
                horizontalSlide.teleOpHorizontalSlide(-1, 2);
            }
            close = false;
            telemetry.addData("close", 6);
        }
        telemetry.addData("close", close);
        return close;
    }
    double placeState = 0;
    double startTimePlace = 0;
    public void placeSample(VerticalGrabberActions verticalGrabber, VerticalWristActions verticalWrist, VerticalSlideActions verticalSlide, HorizontalWristActions horizontalWrist, HorizontalSlideActions horizontalSlide, HorizontalIntakeActions intake) {
        if (placeState == 0) {
            if (close(verticalGrabber, verticalWrist, verticalSlide, horizontalWrist, horizontalSlide)){
                placeState = 1;
            }
        } else if (placeState == 1) {
            verticalGrabber.close();
            intake.close();
            startTimePlace = System.currentTimeMillis();
            placeState = 2;
        } else if (placeState == 2) {
            intake.close();
            if (System.currentTimeMillis() > startTimePlace + 420) {
                placeState = 3;
                verticalWrist.autoFlipForwardDown();
                verticalSlide.setSlidePosition(-700, 2000);
            } else {
                intake.close();
            }
        }
    }
    public void resetPlaceState() {
        placeState = 0;
    }
}