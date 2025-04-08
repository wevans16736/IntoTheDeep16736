package org.firstinspires.ftc.teamcode.secondrobot;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.secondrobot.constants.ConfigConstants;

import java.util.ArrayList;


public class LimeSweet {
    Limelight3A lime; Telemetry telemetry;
    double tx, ty, height, verticalFOV, horizontalFOV;
    double verticalDistance, horizontalDistance, xDistance, yDistance;
    int pipeline; ArrayList<Double> point = new ArrayList<>();
    public LimeSweet(HardwareMap hardwareMap, Telemetry telemetry, int pipeline){
        this.telemetry = telemetry;
        this.lime = hardwareMap.get(Limelight3A.class, ConfigConstants.LIME);
        this.pipeline = pipeline;
        lime.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        lime.start(); // This tells Limelight to start looking!
        lime.pipelineSwitch(this.pipeline); // Switch to pipeline number 0

        double[] inputs = {0,0,0,0,0,0,0,0};
        lime.updatePythonInputs(inputs);

        height = 12;
        verticalFOV = 42;
        horizontalFOV = 54.5;
        verticalDistance = 14;
    }

    public ArrayList<Double> scanButter(){
        if(pipeline == 1) {
            LLResult result = lime.getLatestResult();
            if (result != null && result.isValid()) {
                double tx = result.getTx();
                double ty = result.getTy();
                telemetry.addData("Target X degree", tx);
                telemetry.addData("Target Y degree", ty);

                yDistance = ((ty + (verticalFOV / 2)) / verticalFOV) * verticalDistance;
                xDistance = ((tx / 54.5) * (((yDistance / 14) * 6) + 12));

                telemetry.addData("Distance X", xDistance);
                telemetry.addData("Distance Y", yDistance);
                telemetry.update();

                point.clear();
                point.add(xDistance);
                point.add(yDistance);
            } else {
                telemetry.addLine("no target");
                point.clear();
                point.add(0, -1.0);
                point.add(1, -1.0);
            }
            return point;
        } else {
            // Sending numbers to Python
            double[] inputs = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0};
            lime.updatePythonInputs(inputs);
            LLResult result = lime.getLatestResult();
                // Getting numbers from Python
                double[] pythonOutputs = result.getPythonOutput();
                if (pythonOutputs != null && pythonOutputs.length > 0 && pythonOutputs[3] != 0) {
                    point.clear();
                    point.add(pythonOutputs[2]);
                    point.add(pythonOutputs[3] + 25);
                    point.add(pythonOutputs[4]);
                    telemetry.addData("Python output X:", point.get(0));
                    telemetry.addData("Python output Y:", point.get(1));
                    telemetry.addData("Python output angle:", point.get(2));
                } else {
                    telemetry.addLine("no target");
                    point.clear();
                    point.add(0, -1.0);
                    point.add(1, -1.0);
                    point.add(2, -1.0);
                }
                telemetry.update();
            return point;
        }
    }
}
