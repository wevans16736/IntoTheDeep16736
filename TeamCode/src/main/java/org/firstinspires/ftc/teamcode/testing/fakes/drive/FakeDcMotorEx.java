package org.firstinspires.ftc.teamcode.testing.fakes.drive;
/*
 Copyright (c) 2021 The Tech Ninja Team (https://ftc9929.com)

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
 */



import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.HashMap;
import java.util.Map;

@SuppressWarnings("unused")
public class FakeDcMotorEx implements DcMotorEx {
    private boolean motorEnable = true;

    private Map<RunMode, PIDCoefficients> pidCoefficients = new HashMap<>();

    private Map<RunMode, PIDFCoefficients> pidfCoefficients = new HashMap<>();

    private double motorPower;

    private int currentEncoderPosition;

    private int targetPositionTolerance;

    private double simpleVelocity;
    
    private int targetPosition;

    private boolean isBusy;

    private RunMode runMode;

    public FakeDcMotorEx() {

    }

    @Override
    public void setMotorEnable() {
        motorEnable = true;
    }

    @Override
    public void setMotorDisable() {
        motorEnable = false;
    }

    @Override
    public boolean isMotorEnabled() {
        return motorEnable;
    }

    @Override
    public void setVelocity(double angularRate) {
        simpleVelocity = angularRate;
    }

    @Override
    public void setVelocity(double angularRate, AngleUnit unit) {
        throw new IllegalArgumentException("Not implemented");
    }

    @Override
    public double getVelocity(AngleUnit unit) {
        throw new IllegalArgumentException("Not implemented");
    }

    @Override
    public void setPIDCoefficients(RunMode mode, PIDCoefficients pidCoefficients) {
        this.pidCoefficients.put(mode, pidCoefficients);
    }

    @Override
    public void setPIDFCoefficients(RunMode mode, PIDFCoefficients pidfCoefficients) throws UnsupportedOperationException {
        this.pidfCoefficients.put(mode, pidfCoefficients);
    }

    @Override
    public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {

    }

    @Override
    public void setPositionPIDFCoefficients(double p) {

    }

    @Override
    public PIDCoefficients getPIDCoefficients(RunMode mode) {
        return pidCoefficients.get(mode);
    }

    @Override
    public double getCurrent(CurrentUnit unit) {
        return 0;
    }

    @Override
    public double getCurrentAlert(CurrentUnit unit) {
        return 0;
    }

    @Override
    public void setCurrentAlert(double current, CurrentUnit unit) {

    }

    @Override
    public boolean isOverCurrent() {
        return false;
    }

    @Override
    public PIDFCoefficients getPIDFCoefficients(RunMode mode) {
        return pidfCoefficients.get(mode);
    }

    @Override
    public void setTargetPositionTolerance(int tolerance) {
        targetPositionTolerance = tolerance;
    }

    @Override
    public int getTargetPositionTolerance() {
        return targetPositionTolerance;
    }

    @Override
    public MotorConfigurationType getMotorType() {
        return null;
    }

    @Override
    public void setMotorType(MotorConfigurationType motorType) {

    }

    @Override
    public DcMotorController getController() {
        return null;
    }

    @Override
    public int getPortNumber() {
        return 0;
    }

    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {

    }

    @Override
    public ZeroPowerBehavior getZeroPowerBehavior() {
        return null;
    }

    @Override
    public void setPowerFloat() {

    }

    @Override
    public double getVelocity(){
        return simpleVelocity;
    }

    @Override
    public boolean getPowerFloat() {
        return false;
    }

    @Override
    public void setTargetPosition(int position) {
        targetPosition = position;
    }

    @Override
    public int getTargetPosition() {
        return targetPosition;
    }

    @Override
    public boolean isBusy() {
        return isBusy;
    }

    public void setBusy(final boolean isBusy) {
        this.isBusy = isBusy;
    }

    @Override
    public int getCurrentPosition() {
        return currentEncoderPosition;
    }

    public void setCurrentPosition(int currentMotorPosition) {
        this.currentEncoderPosition = currentMotorPosition;
    }

    @Deprecated
    public void setCurrentPosistion(int currentMotorPosistion) {
        this.currentEncoderPosition = currentMotorPosistion;
    }

    @Override
    public void setMode(RunMode mode) {
        runMode = mode;

        isBusy = mode == RunMode.RUN_TO_POSITION;
    }

    @Override
    public RunMode getMode() {
        return runMode;
    }

    @Override
    public void setDirection(Direction direction) {

    }

    @Override
    public Direction getDirection() {
        return null;
    }

    @Override
    public void setPower(double power) {
        this.motorPower = power;
    }

    @Override
    public double getPower() {
        return motorPower;
    }

    @Override
    public Manufacturer getManufacturer() {
        return null;
    }

    @Override
    public String getDeviceName() {
        return null;
    }

    @Override
    public String getConnectionInfo() {
        return null;
    }

    @Override
    public int getVersion() {
        return 0;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {

    }

    @Override
    public void close() {

    }
}
