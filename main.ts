// Code is based on FastIMU library: https://github.com/LiquidCGS/FastIMU
namespace MintsparkMpu6050{
    // MPU-6050 registers
    // https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf
    // https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
    let MPU6050_SELF_TEST_X_ACCEL = 0x0D;
    let MPU6050_WHO_AM_I_MPU6050 = 0x75; 
    let MPU6050_PWR_MGMT_1 = 0x6B;
    let MPU6050_PWR_MGMT_2 = 0x6C;
    let MPU6050_ADDRESS = 0x68;
    let MPU6050_MPU_CONFIG = 0x1A;
    let MPU6050_SMPLRT_DIV = 0x19;
    let MPU6050_GYRO_CONFIG = 0x1B;
    let MPU6050_ACCEL_CONFIG = 0x1C;
    let MPU6050_INT_PIN_CFG = 0x37;
    let MPU6050_INT_ENABLE = 0x38;
    let MPU6050_INT_STATUS = 0x3A;
    let MPU6050_ACCEL_XOUT_H = 0x3B;
    let MPU6050_FIFO_EN = 0x23;
    let MPU6050_I2C_MST_CTRL = 0x24;
    let MPU6050_USER_CTRL = 0x6A;
    let MPU6050_FIFO_COUNTH = 0x72;
    let MPU6050_FIFO_R_W = 0x74;

    let q: number[] = [1.0, 0.0, 0.0, 0.0]; // vector to hold quaternion
    let Kp = 30.0; // Free parameters in the Mahony filter and fusion scheme,
    let Ki = 0.0;  // Kp for proportional feedback, Ki for integral
    let lastUpdate: number = 0;
    let roll: number, pitch: number, yaw: number = 0;
    let rollDrift: number, pitchDrift: number, yawDrift: number = 0;
    let gXDrift: number, gYDrift: number, gZDrift: number = 0;
    let calibrate = false;
    let calgXSum = 0;
    let calgYSum = 0;
    let calgZSum = 0;
    let calCounter = 0;
    let init = 0
    let aRes = 16.0 / 32768.0;
    let gRes = 250.0 / 32768.0;
    let temperature = 0.0;
    let accel: AccelData = { accelX: 0, accelY: 0, accelZ: 0 };
    let gyro: GyroData = { gyroX: 0, gyroY: 0, gyroZ: 0 };
    let calibration: CalData = { accelBias: [0, 0, 0], gyroBias: [0, 0, 0], magBias: [0, 0, 0], magScale: [0, 0, 0], valid: true };
    let geometryIndex = 0;

    type AccelData = {
        accelX: number;
        accelY: number;
        accelZ: number;
    };

    type GyroData = {
        gyroX: number;
        gyroY: number;
        gyroZ: number;
    };

    type GyroDrift = {
        gyroX: number;
        gyroY: number;
        gyroZ: number;
    };

    type CalData = {
        valid: boolean;
        accelBias: number[];
        gyroBias: number[];
        magBias: number[];
        magScale: number[];
    };



    if (InitMPU6050()) {
        CalibrateAccelGyro();
        setAccelRange(2);
        setGyroRange(500);
        basic.pause(100);
        init = 1
    }

    basic.forever(function () {
        if (init == 1) {
            lastUpdate = input.runningTime();
            UpdateMPU6050();
            //serial.writeValue("gx", gyro.gyroX);
            //serial.writeValue("gy", gyro.gyroY);
            //serial.writeValue("gz", gyro.gyroZ);
        }
    })


    export function InitMPU6050(): boolean {
        // Check device is connected
        if (!(readByte(MPU6050_WHO_AM_I_MPU6050) == MPU6050_ADDRESS)) {
            return false;
        }

        // Reset device
        writeByte(MPU6050_PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
        basic.pause(100);

        // wake up device
        writeByte(MPU6050_PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
        basic.pause(100); // Wait for all registers to reset

        // get stable time source
        writeByte(MPU6050_PWR_MGMT_1, 0x03);  // Auto select clock source to be PLL gyroscope reference if ready
        basic.pause(200);

        // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
        writeByte(MPU6050_SMPLRT_DIV, 0x03);  // Use a 250 Hz rate; a rate consistent with the filter update rate
        // determined inset in CONFIG above

        // Set gyroscope full scale range
        setGyroRange(2000);

        // Set accelerometer full-scale range configuration
        setAccelRange(16);

        // Set accelerometer sample rate configuration
        // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
        writeByte(MPU6050_MPU_CONFIG, 0x03); // Set accelerometer rate to 1 kHz and bandwidth to 44 Hz

        // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
        // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

        // Configure Interrupts and Bypass Enable
        // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
        // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
        // can join the I2C bus and all can be controlled by the Arduino as master
        writeByte(MPU6050_INT_PIN_CFG, 0x22);
        writeByte(MPU6050_INT_ENABLE, 0x01);    // Enable data ready (bit 0) interrupt
        basic.pause(100);

        return true;
    }

    export function UpdateMPU6050() {
        if (!dataAvailable()) return;

        let IMUCount: number[] = []; // used to read all 14 bytes at once from the MPU6050 accel/gyro
        // x/y/z accel register data stored here

        let rawData = readBytes(MPU6050_ACCEL_XOUT_H, 14);    // Read the 14 raw data registers into data array

        IMUCount[0] = combineBytes(rawData[0], rawData[1]);		  // Turn the MSB and LSB into a signed 16-bit value
        IMUCount[1] = combineBytes(rawData[2], rawData[3]);
        IMUCount[2] = combineBytes(rawData[4], rawData[5]);
        IMUCount[3] = combineBytes(rawData[6], rawData[7]);
        IMUCount[4] = combineBytes(rawData[8], rawData[9]);
        IMUCount[5] = combineBytes(rawData[10], rawData[11]);
        IMUCount[6] = combineBytes(rawData[12], rawData[13]);

        // Calculate the accel value into actual g's per second
        let ax = IMUCount[0] * aRes - calibration.accelBias[0];
        let ay = IMUCount[1] * aRes - calibration.accelBias[1];
        let az = IMUCount[2] * aRes - calibration.accelBias[2];

        // Calculate the temperature value into actual deg c
        let temperature = ((IMUCount[3] / 340) + 36.53);

        // Calculate the gyro value into actual degrees per second
        let gx = IMUCount[4] * gRes - calibration.gyroBias[0];
        let gy = IMUCount[5] * gRes - calibration.gyroBias[1];
        let gz = IMUCount[6] * gRes - calibration.gyroBias[2];

        // Set values for geometry index
        switch (geometryIndex) {
            case 0:
                accel.accelX = ax; gyro.gyroX = gx;
                accel.accelY = ay; gyro.gyroY = gy;
                accel.accelZ = az; gyro.gyroZ = gz;
                break;
            case 1:
                accel.accelX = -ay; gyro.gyroX = -gy;
                accel.accelY = ax; gyro.gyroY = gx;
                accel.accelZ = az; gyro.gyroZ = gz;
                break;
            case 2:
                accel.accelX = -ax; gyro.gyroX = -gx;
                accel.accelY = -ay; gyro.gyroY = -gy;
                accel.accelZ = az; gyro.gyroZ = gz;
                break;
            case 3:
                accel.accelX = ay; gyro.gyroX = gy;
                accel.accelY = -ax; gyro.gyroY = -gx;
                accel.accelZ = az; gyro.gyroZ = gz;
                break;
            case 4:
                accel.accelX = -az; gyro.gyroX = -gz;
                accel.accelY = -ay; gyro.gyroY = -gy;
                accel.accelZ = -ax; gyro.gyroZ = -gx;
                break;
            case 5:
                accel.accelX = -az; gyro.gyroX = -gz;
                accel.accelY = ax; gyro.gyroY = gx;
                accel.accelZ = -ay; gyro.gyroZ = -gy;
                break;
            case 6:
                accel.accelX = -az; gyro.gyroX = -gz;
                accel.accelY = ay; gyro.gyroY = gy;
                accel.accelZ = ax; gyro.gyroZ = gx;
                break;
            case 7:
                accel.accelX = -az; gyro.gyroX = -gz;
                accel.accelY = -ax; gyro.gyroY = -gx;
                accel.accelZ = ay; gyro.gyroZ = gy;
                break;
        }

        // Catch elapsed time since last update for integration
        let now = input.runningTime();
        let deltat = (now - lastUpdate) / 1000; //seconds since last update
        lastUpdate = now;

        // Integrate to calc pitch, roll, yaw (yaw has an empirical division by PI to provide correct degree values but not sure why :-) )
        // Code based on: https://github.com/jremington/MPU-6050-Fusion/
        // Conversions: http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        // WARNING: This angular conversion is for DEMONSTRATION PURPOSES ONLY. It WILL MALFUNCTION for certain combinations of angles! See https://en.wikipedia.org/wiki/Gimbal_lock
        Mahony_update_full(accel.accelX, accel.accelY, accel.accelZ, gyro.gyroX, gyro.gyroY, gyro.gyroZ / Math.PI, deltat);
        roll = Math.atan2((q[0] * q[1] + q[2] * q[3]), 0.5 - (q[1] * q[1] + q[2] * q[2]));
        pitch = Math.asin(2.0 * (q[0] * q[2] - q[1] * q[3]));
        yaw = -Math.atan2((q[1] * q[2] + q[0] * q[3]), 0.5 - (q[2] * q[2] + q[3] * q[3]));
        yaw *= 180.0 / Math.PI;
        if (yaw < 0) yaw += 360.0; //compass circle
        pitch *= 180.0 / Math.PI;
        roll *= 180.0 / Math.PI;

        // Collect data for calibration if calibration is running
        if (calibrate) {
            calgXSum += gx;
            calgYSum += gy;
            calgZSum += gz;
            calCounter++;
        }
    }

    export function CalibrateAccelGyro() {
        let data: number[];
        let ii, packet_count, fifo_count: number;
        let gyro_bias: number[] = [0, 0, 0];
        let accel_bias: number[] = [0, 0, 0];

        // Capture current settings to apply after calibration is complete
        let oldMpu = readByte(MPU6050_MPU_CONFIG);
        let oldSpl = readByte(MPU6050_SMPLRT_DIV);
        let oldGyro = readByte(MPU6050_GYRO_CONFIG);
        let oldAccel = readByte(MPU6050_ACCEL_CONFIG);

        // reset device
        writeByte(MPU6050_PWR_MGMT_1, 0x80);
        basic.pause(100);

        // get stable time source; Auto select clock source to be PLL Z axis gyroscope reference if ready
        writeByte(MPU6050_PWR_MGMT_1, 0x03);
        basic.pause(100);

        // Configure device for bias calculation
        writeByte(MPU6050_INT_ENABLE, 0x00);   // Disable all interrupts
        writeByte(MPU6050_FIFO_EN, 0x00);      // Disable FIFO
        writeByte(MPU6050_I2C_MST_CTRL, 0x00); // Disable I2C master
        writeByte(MPU6050_USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
        writeByte(MPU6050_USER_CTRL, 0x04);    // Reset FIFO
        basic.pause(15);

        // Configure MPU6050 gyro and accelerometer for bias calculation
        writeByte(MPU6050_MPU_CONFIG, 0x01);  // Set low-pass filter to 188 Hz
        writeByte(MPU6050_SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
        writeByte(MPU6050_GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
        writeByte(MPU6050_ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity
        let gyrosensitivity = 131;   // = 131 LSB/degrees/sec
        let accelsensitivity = 16384;  // = 16384 LSB/g

        // Configure FIFO to capture accelerometer and gyro data for bias calculation
        writeByte(MPU6050_USER_CTRL, 0x40);   // Enable FIFO
        writeByte(MPU6050_FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
        basic.pause(40); // accumulate 40 samples in 40 milliseconds = 480 bytes
        writeByte(MPU6050_FIFO_EN, 0x00); // Disable gyro and accelerometer sensors for FIFO

        let buffer1 = readBytes(MPU6050_FIFO_COUNTH, 2); // read FIFO sample count
        fifo_count = combineBytes(buffer1[0], buffer1[1]);
        packet_count = fifo_count / 12; // Number of sets of full gyro and accelerometer data for averaging

        for (ii = 0; ii < packet_count; ii++) {
            let accel_temp: number[] = [0, 0, 0];
            let gyro_temp: number[] = [0, 0, 0];

            let buffer2 = readBytes(MPU6050_FIFO_R_W, 12); // read data for averaging
            accel_temp[0] = combineBytes(buffer2[0], buffer2[1]);
            accel_temp[1] = combineBytes(buffer2[2], buffer2[3]);
            accel_temp[2] = combineBytes(buffer2[4], buffer2[5]);
            gyro_temp[0] = combineBytes(buffer2[6], buffer2[7]);
            gyro_temp[1] = combineBytes(buffer2[8], buffer2[9]);
            gyro_temp[2] = combineBytes(buffer2[10], buffer2[11]);

            accel_bias[0] += accel_temp[0]; // Sum individual signed biases
            accel_bias[1] += accel_temp[1];
            accel_bias[2] += accel_temp[2];
            gyro_bias[0] += gyro_temp[0];
            gyro_bias[1] += gyro_temp[1];
            gyro_bias[2] += gyro_temp[2];
        }

        accel_bias[0] /= packet_count;
        accel_bias[1] /= packet_count;
        accel_bias[2] /= packet_count;
        gyro_bias[0] /= packet_count;
        gyro_bias[1] /= packet_count;
        gyro_bias[2] /= packet_count;

        switch (geometryIndex) {
            case 0:
            case 1:
            case 2:
            case 3:
                if (accel_bias[2] > 0) {
                    accel_bias[2] -= accelsensitivity; // Remove gravity from the z-axis accelerometer bias calculation
                }
                else {
                    accel_bias[2] += accelsensitivity;
                }
                break;
            case 4:
            case 6:
                if (accel_bias[0] > 0) {
                    accel_bias[0] -= accelsensitivity; // Remove gravity from the z-axis accelerometer bias calculation
                }
                else {
                    accel_bias[0] += accelsensitivity;
                }
                break;
            case 5:
            case 7:
                if (accel_bias[1] > 0) {
                    accel_bias[1] -= accelsensitivity; // Remove gravity from the z-axis accelerometer bias calculation
                }
                else {
                    accel_bias[1] += accelsensitivity;
                }
                break;
        }

        // Calc scaled accelerometer and gyro biases
        let accelBias0 = accel_bias[0] / accelsensitivity;
        let accelBias1 = accel_bias[1] / accelsensitivity;
        let accelBias2 = accel_bias[2] / accelsensitivity;
        let gyroBias0 = gyro_bias[0] / gyrosensitivity;
        let gyroBias1 = gyro_bias[1] / gyrosensitivity;
        let gyroBias2 = gyro_bias[2] / gyrosensitivity;

        calibration = { accelBias: [accelBias0, accelBias1, accelBias2], gyroBias: [gyroBias0, gyroBias1, gyroBias2], magBias: [0, 0, 0], magScale: [0, 0, 0], valid: true };

        // Return settings to original state
        writeByte(MPU6050_MPU_CONFIG, oldMpu);
        writeByte(MPU6050_SMPLRT_DIV, oldSpl);
        writeByte(MPU6050_GYRO_CONFIG, oldGyro);
        writeByte(MPU6050_ACCEL_CONFIG, oldAccel);
        basic.pause(200);
    }

    export function setAccelRange(range: number) {
        let c: number;
        if (range == 16) {
            aRes = 16 / 32768;	// full range (16g) readings
            c = 0x03 << 3;
        }
        else if (range == 8) {
            aRes = 8 / 32768;	// range (8g) readings
            c = 0x02 << 3;
        }
        else if (range == 4) {
            aRes = 4 / 32768;	// range (4g) readings
            c = 0x01 << 3;
        }
        else if (range == 2) {
            aRes = 2 / 32768;	// range (2g) readings
            c = 0x00 << 3;
        }

        writeByte(MPU6050_ACCEL_CONFIG, c);
    }

    export function setGyroRange(range: number) {
        let c;
        if (range == 2000) {
            gRes = 2000 / 32768;	// full range (2000dps) readings
            c = 0x03 << 3;
        }
        else if (range == 1000) {
            gRes = 1000 / 32768;	// range (1000dps) readings
            c = 0x02 << 3;
        }
        else if (range == 500) {
            gRes = 500 / 32768;		// range (500dps) readings
            c = 0x01 << 3;
        }
        else if (range == 250) {
            gRes = 250 / 32768;		// range (250dps) readings
            c = 0x00 << 3;
        }

        writeByte(MPU6050_GYRO_CONFIG, c); // Write new GYRO_CONFIG register value
    }

    export function calibrateGyros(seconds: number) {
        let startTime = input.runningTime();
        calgXSum = 0, calgYSum = 0, calgZSum = 0, calCounter = 0;

        // Start calibration
        calibrate = true;
        basic.pause(seconds * 1000);
        calibrate = false;

        // Calculate drift
        gXDrift = calgXSum / calCounter;
        gYDrift = calgYSum / calCounter;
        gZDrift = calgZSum / calCounter;

        calibration.gyroBias[0] += gXDrift;
        calibration.gyroBias[1] += gYDrift;
        calibration.gyroBias[2] += gZDrift;
    }

    function combineBytes(h: number, l: number): number {
        let value = (h << 8) + l;

        if (value >= 0x8000) {
            return -((65535 - value) + 1);
        }
        else {
            return value;
        }
    }

    function readByte(reg: number): NumberFormat.UInt8BE {
        pins.i2cWriteNumber(MPU6050_ADDRESS, reg, NumberFormat.UInt8BE);
        let value = pins.i2cReadNumber(MPU6050_ADDRESS, NumberFormat.UInt8BE);
        return value;
    }

    function readBytes(reg: number, size: number): Buffer {
        pins.i2cWriteNumber(MPU6050_ADDRESS, reg, NumberFormat.UInt8BE);
        let value = pins.i2cReadBuffer(MPU6050_ADDRESS, size);
        return value;
    }

    function writeByte(reg: number, value: number) {
        let buffer = pins.createBuffer(2);
        buffer[0] = reg;
        buffer[1] = value;
        pins.i2cWriteBuffer(MPU6050_ADDRESS, buffer);
    }

    function dataAvailable(): boolean {
        return readByte(MPU6050_INT_STATUS) == 1;
    }

    // Code based on: https://github.com/jremington/MPU-6050-Fusion/
    function Mahony_update_full(ax: number, ay: number, az: number, gx: number, gy: number, gz: number, deltat: number) {
        let recipNorm;
        let vx, vy, vz;
        let ex, ey, ez;  //error terms
        let qa, qb, qc;
        let ix = 0.0, iy = 0.0, iz = 0.0;  //integral feedback terms
        let tmp;

        // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
        tmp = ax * ax + ay * ay + az * az;
        if (tmp > 0.0) {

            // Normalise accelerometer (assumed to measure the direction of gravity in body frame)
            recipNorm = 1.0 / Math.sqrt(tmp);
            ax *= recipNorm;
            ay *= recipNorm;
            az *= recipNorm;

            // Estimated direction of gravity in the body frame (factor of two divided out)
            vx = q[1] * q[3] - q[0] * q[2];  //to normalize these terms, multiply each by 2.0
            vy = q[0] * q[1] + q[2] * q[3];
            vz = q[0] * q[0] - 0.5 + q[3] * q[3];

            // Error is cross product between estimated and measured direction of gravity in body frame
            // (half the actual magnitude)
            ex = (ay * vz - az * vy);
            ey = (az * vx - ax * vz);
            ez = (ax * vy - ay * vx);

            // Compute and apply to gyro term the integral feedback, if enabled
            if (Ki > 0.0) {
                ix += Ki * ex * deltat;  // integral error scaled by Ki
                iy += Ki * ey * deltat;
                iz += Ki * ez * deltat;
                gx += ix;  // apply integral feedback
                gy += iy;
                gz += iz;
            }

            // Apply proportional feedback to gyro term
            gx += Kp * ex;
            gy += Kp * ey;

            serial.writeValue("gz1", gz);
            gz += Kp * ez;
            serial.writeValue("gz2", gz);
        }

        // Integrate rate of change of quaternion, q cross gyro term
        deltat = 0.5 * deltat;
        gx *= deltat;   // pre-multiply common factors
        gy *= deltat;
        gz *= deltat;
        qa = q[0];
        qb = q[1];
        qc = q[2];
        q[0] += (-qb * gx - qc * gy - q[3] * gz);
        q[1] += (qa * gx + qc * gz - q[3] * gy);
        q[2] += (qa * gy - qb * gz + q[3] * gx);
        q[3] += (qa * gz + qb * gy - qc * gx);

        // renormalise quaternion
        recipNorm = 1.0 / Math.sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
        q[0] = q[0] * recipNorm;
        q[1] = q[1] * recipNorm;
        q[2] = q[2] * recipNorm;
        q[3] = q[3] * recipNorm;
    }
}