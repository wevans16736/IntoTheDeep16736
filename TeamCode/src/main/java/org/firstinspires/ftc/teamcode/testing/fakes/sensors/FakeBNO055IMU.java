package org.firstinspires.ftc.teamcode.testing.fakes.sensors;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.TimestampedData;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.MagneticFlux;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.Temperature;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

@SuppressWarnings("unused")
public class FakeBNO055IMU implements BNO055IMU {

    public FakeBNO055IMU() {
        super();
    }

    public final static byte bCHIP_ID_VALUE = (byte)0xa0;

    Orientation orientation;

    enum VECTOR
    {
        ACCELEROMETER   (Register.ACC_DATA_X_LSB),
        MAGNETOMETER    (Register.MAG_DATA_X_LSB),
        GYROSCOPE       (Register.GYR_DATA_X_LSB),
        EULER           (Register.EUL_H_LSB),
        LINEARACCEL     (Register.LIA_DATA_X_LSB),
        GRAVITY         (Register.GRV_DATA_X_LSB);
        //------------------------------------------------------------------------------------------
        protected byte value;
        VECTOR(int value) { this.value = (byte)value; }
        VECTOR(Register register) { this(register.bVal); }
        public byte getValue() { return this.value; }
    }

    enum POWER_MODE
    {
        NORMAL(0X00),
        LOWPOWER(0X01),
        SUSPEND(0X02);
        //------------------------------------------------------------------------------------------
        protected byte value;
        POWER_MODE(int value) { this.value = (byte)value; }
        public byte getValue() { return this.value; }
    }

    protected static class VectorData
    {
        public TimestampedData data;
        public    float             scale;
        protected ByteBuffer buffer;

        public VectorData(TimestampedData data, float scale)
        {
            this.data = data;
            this.scale = scale;
            buffer = ByteBuffer.wrap(data.data).order(ByteOrder.LITTLE_ENDIAN);
        }

        public float next()
        {
            return buffer.getShort() / scale;
        }
    }

    @Override
    public boolean initialize(@NonNull Parameters parameters) {
        return true;
    }

    @NonNull
    @Override
    public Parameters getParameters() {
        throw new IllegalArgumentException("Not implemented");
    }

    @Override
    public void close() {
        throw new IllegalArgumentException("Not implemented");
    }

    @Override
    public Orientation getAngularOrientation() {
        return orientation;
    }

    public void setAngularOrientation(float angle){
        this.orientation = new Orientation(AxesReference.INTRINSIC, AxesOrder.ZYX, org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES,
                angle,
                0,
                0,
                0);
    }

    @Override
    public Orientation getAngularOrientation(AxesReference reference, AxesOrder order, org.firstinspires.ftc.robotcore.external.navigation.AngleUnit angleUnit) {
        return orientation;
    }

    @Override
    public Acceleration getOverallAcceleration() {
        throw new IllegalArgumentException("Not implemented");
    }

    @Override
    public AngularVelocity getAngularVelocity() {
        throw new IllegalArgumentException("Not implemented");
    }

    @Override
    public Acceleration getLinearAcceleration() {
        throw new IllegalArgumentException("Not implemented");
    }

    @Override
    public Acceleration getGravity() {
        throw new IllegalArgumentException("Not implemented");
    }

    @Override
    public Temperature getTemperature() {
        throw new IllegalArgumentException("Not implemented");
    }

    @Override
    public MagneticFlux getMagneticFieldStrength() {
        throw new IllegalArgumentException("Not implemented");
    }

    @Override
    public Quaternion getQuaternionOrientation() {
        throw new IllegalArgumentException("Not implemented");
    }

    @Override
    public Position getPosition() {
        throw new IllegalArgumentException("Not implemented");
    }

    @Override
    public Velocity getVelocity() {
        throw new IllegalArgumentException("Not implemented");
    }

    @Override
    public Acceleration getAcceleration() {
        throw new IllegalArgumentException("Not implemented");
    }

    @Override
    public void startAccelerationIntegration(Position initialPosition, Velocity initialVelocity, int msPollInterval) {
        throw new IllegalArgumentException("Not implemented");
    }

    @Override
    public void stopAccelerationIntegration() {
        throw new IllegalArgumentException("Not implemented");
    }

    @Override
    public SystemStatus getSystemStatus() {
        throw new IllegalArgumentException("Not implemented");
    }

    @Override
    public SystemError getSystemError() {
        throw new IllegalArgumentException("Not implemented");
    }

    @Override
    public CalibrationStatus getCalibrationStatus() {
        throw new IllegalArgumentException("Not implemented");
    }

    @Override
    public boolean isSystemCalibrated() {
        throw new IllegalArgumentException("Not implemented");
    }

    @Override
    public boolean isGyroCalibrated() {
        throw new IllegalArgumentException("Not implemented");
    }

    @Override
    public boolean isAccelerometerCalibrated() {
        throw new IllegalArgumentException("Not implemented");
    }

    @Override
    public boolean isMagnetometerCalibrated() {
        throw new IllegalArgumentException("Not implemented");
    }

    @Override
    public CalibrationData readCalibrationData() {
        throw new IllegalArgumentException("Not implemented");
    }

    @Override
    public void writeCalibrationData(CalibrationData data) {
        throw new IllegalArgumentException("Not implemented");
    }

    @Override
    public byte read8(Register register) {
        throw new IllegalArgumentException("Not implemented");
    }

    @Override
    public byte[] read(Register register, int cb) {
        throw new IllegalArgumentException("Not implemented");
    }

    @Override
    public void write8(Register register, int bVal) {
        throw new IllegalArgumentException("Not implemented");
    }

    @Override
    public void write(Register register, byte[] data) {
        throw new IllegalArgumentException("Not implemented");
    }

}
