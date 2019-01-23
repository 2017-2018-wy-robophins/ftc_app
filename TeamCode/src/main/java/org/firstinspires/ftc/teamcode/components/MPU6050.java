package org.firstinspires.ftc.teamcode.components;

import android.support.annotation.NonNull;

import com.qualcomm.robotcore.hardware.AccelerationSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDeviceWithParameters;
import com.qualcomm.robotcore.hardware.configuration.I2cSensor;
import com.qualcomm.robotcore.util.TypeConversion;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;

//Code for implementation of the arm gyroscope.
@I2cSensor(name = "MPU6050 Accelerometer + Gyroscope", description = "Gyroscope from Invensense", xmlTag = "MPU6050")
public class MPU6050 extends I2cDeviceSynchDeviceWithParameters<I2cDeviceSynch, MPU6050.Parameters> {

    private static final I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create7bit(0x68);
    private double GYRO_SCALAR;
    private double ACCEL_SCALAR;

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    public static class Parameters implements Cloneable
    {
        I2cAddr i2cAddr = ADDRESS_I2C_DEFAULT;

        // All settings available
        EXT_SYNC_SET ext_sync_set = EXT_SYNC_SET.DISABLED;
        DLPF_CFG dlpf_cfg = DLPF_CFG.DLPF_CFG_1;
        FS_SEL fs_sel = FS_SEL.FS_SEL_250_DEG_S;
        AFS_SEL afs_sel = AFS_SEL.AFS_SEL_4G;
        TEMP_DIS temp_dis = TEMP_DIS.DISABLE;
        CLKSEL clksel = CLKSEL.PLL_X_AXIS_GYRO;

        public Parameters clone()
        {
            try
            {
                return (Parameters) super.clone();
            }
            catch(CloneNotSupportedException e)
            {
                throw new RuntimeException("Internal Error: Parameters not cloneable");
            }
        }
    }

    @Override
    protected synchronized boolean internalInitialize(@NonNull Parameters params) {
        this.parameters = params.clone();

        GYRO_SCALAR = params.fs_sel.toGyroScalar();
        ACCEL_SCALAR = params.afs_sel.toAccelScalar();

        deviceClient.setI2cAddress(params.i2cAddr);

        boolean resetOk = resetDevice();

        int configSettings = params.ext_sync_set.bVal | params.dlpf_cfg.bVal;
        writeByte(Register.CONFIG, (byte)configSettings);
        boolean configOk = (readByte(Register.CONFIG) & 0x3F) == configSettings;

        int gyroConfigSettings = params.fs_sel.bVal;
        writeByte(Register.GYRO_CONFIG, (byte)gyroConfigSettings);
        boolean gyroConfigOk = (readByte(Register.GYRO_CONFIG) & 0x18) == gyroConfigSettings;

        int accelConfigSettings = params.afs_sel.bVal;
        writeByte(Register.ACCEL_CONFIG, (byte)accelConfigSettings);
        boolean accelConfigOk = (readByte(Register.ACCEL_CONFIG) & 0x18) == accelConfigSettings;

        int userCtrlSettings = params.temp_dis.bVal | params.clksel.bVal;
        writeByte(Register.PWR_MGMT_1, (byte)userCtrlSettings);
        boolean userCtrlOk = (readByte(Register.PWR_MGMT_1) & 0xF) == userCtrlSettings;

        return resetOk && configOk && gyroConfigOk && accelConfigOk && userCtrlOk;
    }

    protected synchronized boolean resetDevice() {
        int resetSetting = DEVICE_RESET.RESET.bVal;
        writeByte(Register.PWR_MGMT_1, (byte)resetSetting);
        boolean resetComplete = false;
        boolean successfulReset = true;
        while (!resetComplete) {
            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                System.err.println("Device reset interuppted.");
                successfulReset = false;
                break;
            }
            resetComplete = ((readByte(Register.PWR_MGMT_1) & 0x80) >>> 7) == 1;
        }
        return successfulReset;
    }

    @Override
    public String getDeviceName() {
        return "MPU6050 Accelerometer + Gyroscope";
    }

    public MPU6050(I2cDeviceSynch deviceClient) {
        super(deviceClient, true, new Parameters());
        this.setOptimalReadWindow();
        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);

        super.registerArmingStateCallback(false); // deals with usb cables getting unplugged
        this.deviceClient.engage();
    }

    public enum Register {
        FIRST(0),
        CONFIG(0x1A),
        GYRO_CONFIG(0x1B),
        ACCEL_CONFIG(0x1C),
        ACCEL_XOUT_H(0x3B),
        ACCEL_XOUT_L(0x3C),
        ACCEL_YOUT_H(0x3D),
        ACCEL_YOUT_L(0x3E),
        ACCEL_ZOUT_H(0x3F),
        ACCEL_ZOUT_L(0x40),
        GYRO_XOUT_H(0x43),
        GYRO_XOUT_L(0x44),
        GYRO_YOUT_H(0x45),
        GYRO_YOUT_L(0x46),
        GYRO_ZOUT_H(0x47),
        GYRO_ZOUT_L(0x48),
        // USER_CTRL(0x6A),
        PWR_MGMT_1(0x6B),
        WHO_AM_I(0x75),
        LAST(WHO_AM_I.bVal);

        public int bVal;

        Register(int bVal) {
            this.bVal = bVal;
        }
    }

    protected void setOptimalReadWindow() {
        // Sensor registers are read repeatedly and stored in a register. This method specifies the
        // registers and repeat read mode
        I2cDeviceSynch.ReadWindow readWindow = new I2cDeviceSynch.ReadWindow(
                Register.FIRST.bVal,
                Register.LAST.bVal - Register.FIRST.bVal + 1,
                I2cDeviceSynch.ReadMode.REPEAT);
        this.deviceClient.setReadWindow(readWindow);
    }

    protected void writeByte(final Register reg, byte value) {
        deviceClient.write(reg.bVal, new byte[] {value});
    }

    protected byte readByte(Register reg) {
        return deviceClient.read(reg.bVal, 1)[0];
    }

    protected short readShort(Register registerHigh, Register registerLow) {
        return TypeConversion.byteArrayToShort(new byte[] {readByte(registerHigh), readByte(registerLow)});
    }

    public byte getHardwareIDRaw() {
        return readByte(Register.WHO_AM_I);
    }

    public short getAccelXRaw() {
        return readShort(Register.ACCEL_XOUT_H, Register.ACCEL_XOUT_L);
    }

    public short getAccelYRaw() {
        return readShort(Register.ACCEL_YOUT_H, Register.ACCEL_YOUT_L);
    }

    public short getAccelZRaw() {
        return readShort(Register.ACCEL_ZOUT_H, Register.ACCEL_ZOUT_L);
    }

    public short getGyroXRaw() {
        return readShort(Register.GYRO_XOUT_H, Register.GYRO_XOUT_L);
    }

    public short getGyroYRaw() {
        return readShort(Register.GYRO_YOUT_H, Register.GYRO_YOUT_L);
    }

    public short getGyroZRaw() {
        return readShort(Register.GYRO_ZOUT_H, Register.GYRO_ZOUT_L);
    }

    public Acceleration getAcceleration() {
        return Acceleration.fromGravity(ACCEL_SCALAR * getAccelXRaw(), ACCEL_SCALAR * getAccelYRaw(), ACCEL_SCALAR * getAccelZRaw(), System.nanoTime());
    }

    public AngularVelocity getAngularVelocity() {
        return new AngularVelocity(AngleUnit.DEGREES, (float)(GYRO_SCALAR * getGyroXRaw()), (float)(GYRO_SCALAR * getGyroYRaw()), (float)(GYRO_SCALAR * getGyroZRaw()), System.nanoTime());
    }

    public enum DLPF_CFG {
        DLPF_CFG_0(0x0),
        DLPF_CFG_1(0x1),
        DLPF_CFG_2(0x2),
        DLPF_CFG_3(0x3),
        DLPF_CFG_4(0x4),
        DLPF_CFG_5(0x5),
        DLPF_CFG_6(0x6);

        public int bVal;

        DLPF_CFG(int bVal) {
            this.bVal = bVal;
        }
    }

    public enum EXT_SYNC_SET {
        DISABLED(0x0),
        TEMP_OUT_L(0x1),
        GYRO_XOUT_L(0x2),
        GYRO_YOUT_L(0x3),
        GYRO_ZOUT_L(0x4),
        ACCEL_XOUT_L(0x5),
        ACCEL_YOUT_L(0x6),
        ACCEL_ZOUT_L(0x7);

        public int bVal;
        EXT_SYNC_SET(int bVal) {
            this.bVal = bVal << 3;
        }
    }

    public enum FS_SEL {
        FS_SEL_250_DEG_S(0x0), // 250 deg/s
        FS_SEL_500_DEG_S(0x1),
        FS_SEL_1000_DEG_S(0x2),
        FS_SEL_2000_DEG_S(0x3);

        public int bVal;
        private int aVal;
        FS_SEL(int bVal) {
            this.aVal = bVal;
            this.bVal = bVal << 3;
        }

        // deg/s /LSB
        public double toGyroScalar() {
            return Math.pow(2, aVal) / 131;
        }
    }

    public enum AFS_SEL {
        AFS_SEL_2G(0x0),
        AFS_SEL_4G(0x1),
        AFS_SEL_8G(0x2),
        AFS_SEL_16G(0x3);

        public int bVal;
        private int aVal;
        AFS_SEL(int bVal) {
            this.aVal = bVal;
            this.bVal = bVal << 3;
        }

        // g /LSB
        public double toAccelScalar() {
            return Math.pow(2, aVal) / 16384;
        }
    }

    public enum DEVICE_RESET {
        NOOP(0x0),
        RESET(0x1);

        public int bVal;
        DEVICE_RESET(int bVal) {
            this.bVal = bVal << 7;
        }
    }

    public enum TEMP_DIS {
        ENABLE(0x0),
        DISABLE(0x1);

        public int bVal;
        TEMP_DIS(int bVal) {
            this.bVal = bVal << 3;
        }
    }

    public enum CLKSEL {
        INTERNAL(0x0),
        PLL_X_AXIS_GYRO(0x1),
        PLL_Y_AXIS_GYRO(0x2),
        PLL_Z_AXIS_GYRO(0x3),
        PLL_EXTERNAL_32KHZ(0x4),
        PLL_EXTERNAL_19MHZ(0x5),
        STOP(0x7);

        public int bVal;
        CLKSEL(int bval) {
            this.bVal = bVal;
        }
    }
}
