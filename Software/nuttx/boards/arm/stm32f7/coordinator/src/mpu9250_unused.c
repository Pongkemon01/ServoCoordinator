static float fGyroBias[3] = {0, 0, 0}, fAccelBias[3] = {0, 0, 0}; // Bias corrections for gyro and accelerometer
static float fMagnaticBias[3] = {0, 0, 0}, fMagnaticScale[3]  = {0, 0, 0};

/*------------------*/
static int ReadTempData(void)
{
    uint8_t rawData[2];  // x/y/z gyro register data stored here
    int     retval;

    // Read the 6 raw data registers into data array
    retval = MPUReadBytes( TEMP_OUT_H, rawData, 2 );  
    if( retval != OK )
        return retval;  /* Return a negative (a.k.a. error) back */

    return ((int)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a 16-bit value
}


/************************************************************************************
 * Function which accumulates gyro and accelerometer data after device initialization.
 * It calculates the average of the at-rest readings and then loads the resulting
 * offsets into accelerometer and gyro bias registers.
 ************************************************************************************/
static void CalibrateAccelGyro(void)
{  
    uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
    uint16_t ii, packet_count, fifo_count;
    int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
    int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
    uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis
 
    // reset device
    MPUWrite1Byte ( PWR_MGMT_1, 0x80 ); // Write a one to bit 7 reset bit; toggle reset device
    usleep( 10000 );
   
    // get stable time source; Auto select clock source to be PLL gyroscope reference if ready 
    // else use the internal oscillator, bits 2:0 = 001
    MPUWrite1Byte( PWR_MGMT_1, 0x01 );  
    MPUWrite1Byte( PWR_MGMT_2, 0x00 );
    usleep( 20000 );                                    

    // Configure device for bias calculation
    MPUWrite1Byte( INT_ENABLE, 0x00 );   // Disable all interrupts
    MPUWrite1Byte( FIFO_EN, 0x00 );      // Disable FIFO
    MPUWrite1Byte( PWR_MGMT_1, 0x00 );   // Turn on internal clock source
    MPUWrite1Byte( I2C_MST_CTRL, 0x00 ); // Disable I2C master
    MPUWrite1Byte( USER_CTRL, 0x00 );    // Disable FIFO and I2C master modes
    MPUWrite1Byte( USER_CTRL, 0x0C );    // Reset FIFO and DMP
    usleep(1500);
  
    // Configure MPU6050 gyro and accelerometer for bias calculation
    MPUWrite1Byte( CONFIG, 0x01 );      // Set low-pass filter to 188 Hz
    MPUWrite1Byte( SMPLRT_DIV, 0x00 );  // Set sample rate to 1 kHz
    MPUWrite1Byte( GYRO_CONFIG, 0x00 );  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
    MPUWrite1Byte( ACCEL_CONFIG, 0x00 ); // Set accelerometer full-scale to 2 g, maximum sensitivity

    // Configure FIFO to capture accelerometer and gyro data for bias calculation
    MPUWrite1Byte( USER_CTRL, 0x40 );   // Enable FIFO  
    MPUWrite1Byte( FIFO_EN, 0x78 );     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
    usleep(40000); // accumulate 40 samples in 40 milliseconds = 480 bytes

    // At end of sample accumulation, turn off FIFO sensor read
    MPUWrite1Byte( FIFO_EN, 0x00 );        // Disable gyro and accelerometer sensors for FIFO
    MPUReadBytes( FIFO_COUNTH, data, 2 ); // read FIFO sample count
    fifo_count = ((uint16_t)data[0] << 8) | data[1];
    packet_count = fifo_count / 12;// How many sets of full gyro and accelerometer data for averaging
  
    for (ii = 0; ii < packet_count; ii++)
    {
        int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
        MPUReadBytes( FIFO_R_W, data, 12 ); // read data for averaging
        accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
        accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
        accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;    
        gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
        gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
        gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;
    
        accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
        accel_bias[1] += (int32_t) accel_temp[1];
        accel_bias[2] += (int32_t) accel_temp[2];
        gyro_bias[0]  += (int32_t) gyro_temp[0];
        gyro_bias[1]  += (int32_t) gyro_temp[1];
        gyro_bias[2]  += (int32_t) gyro_temp[2];
    }

    accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
    accel_bias[1] /= (int32_t) packet_count;
    accel_bias[2] /= (int32_t) packet_count;
    gyro_bias[0]  /= (int32_t) packet_count;
    gyro_bias[1]  /= (int32_t) packet_count;
    gyro_bias[2]  /= (int32_t) packet_count;
    
    if(accel_bias[2] > 0L) 
    {
        accel_bias[2] -= 16384;  // = 16384 LSB/g
    }  // Remove gravity from the z-axis accelerometer bias calculation
    else 
    {
        accel_bias[2] += 16384;  // = 16384 LSB/g
    }
   
    // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
    data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
    data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
    data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
    data[3] = (-gyro_bias[1]/4)       & 0xFF;
    data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
    data[5] = (-gyro_bias[2]/4)       & 0xFF;
  
    // Push gyro biases to hardware registers
    MPUWrite1Byte( XG_OFFSET_H, data[0] );
    MPUWrite1Byte( XG_OFFSET_L, data[1] );
    MPUWrite1Byte( YG_OFFSET_H, data[2] );
    MPUWrite1Byte( YG_OFFSET_L, data[3] );
    MPUWrite1Byte( ZG_OFFSET_H, data[4] );
    MPUWrite1Byte( ZG_OFFSET_L, data[5] );
  
    // Output scaled gyro biases for display in the main program
    fGyroBias[0] = ( (float)gyro_bias[0] )/ 131.0f;  // = 131 LSB/degrees/sec
    fGyroBias[1] = ( (float)gyro_bias[1] )/ 131.0f;
    fGyroBias[2] = ( (float)gyro_bias[2] )/ 131.0f;

    /*
     * Construct the accelerometer biases for push to the hardware accelerometer
     * bias registers. These registers contain factory trim values which must be
     * added to the calculated accelerometer biases; on boot up these registers 
     * will hold non-zero values. In addition, bit 0 of the lower byte must be 
     * preserved since it is used for temperature compensation calculations. 
     * Accelerometer bias registers expect bias input as 2048 LSB per g, so that
     * the accelerometer biases calculated above must be divided by 8.
     */

    MPUReadBytes( XA_OFFSET_H, data, 2) ; // Read factory accelerometer trim values
    accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
    MPUReadBytes( YA_OFFSET_H, data, 2 );
    accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
    MPUReadBytes( ZA_OFFSET_H, data, 2 );
    accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  
  
    for( ii = 0; ii < 3; ii++ )
    {
        if((accel_bias_reg[ii] & 1)) 
            mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
    }
  
    // Construct total accelerometer bias, including calculated average accelerometer bias from above
    accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
    accel_bias_reg[1] -= (accel_bias[1]/8);
    accel_bias_reg[2] -= (accel_bias[2]/8);
  
    data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
    data[1] = (accel_bias_reg[0])      & 0xFF;
    data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
    data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
    data[3] = (accel_bias_reg[1])      & 0xFF;
    data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
    data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
    data[5] = (accel_bias_reg[2])      & 0xFF;
    data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers
 
    // Apparently this is not working for the acceleration biases in the MPU-9250
    // Are we handling the temperature correction bit properly?
    // Push accelerometer biases to hardware registers
    /*  writeByte(MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
    writeByte(MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
    writeByte(MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
    writeByte(MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
    writeByte(MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
    writeByte(MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);
    */
    // Output scaled accelerometer biases for display in the main program
    fAccelBias[0] = (float)accel_bias[0]/16384.0f;  // = 16384 LSB/g
    fAccelBias[1] = (float)accel_bias[1]/16384.0f;  // = 16384 LSB/g
    fAccelBias[2] = (float)accel_bias[2]/16384.0f;  // = 16384 LSB/g
}

/************************************************************************************
 * Function which fine tune the magnetometer. It requires user action!!!!
 * To start this calibration, user must wave the device in a figure of 8 until 
 * the process is done (about 20 seconds).
 ************************************************************************************/
static void CalibrateMagneto(void) 
{
    float avg_rad;
    uint16_t ii;
    int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
    int16_t mag_max[3] = {-32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_temp[3] = {0, 0, 0};

    //Serial.println("Mag Calibration: Wave device in a figure eight until done!");
    usleep(1000000);
  
    // at 100 Hz ODR, new mag data is available every 10 ms
    for(ii = 0; ii < 1500; ii++) 
    {
        ReadMagData( mag_temp );  // Read the mag data   
        for (int jj = 0; jj < 3; jj++) 
        {
            if(mag_temp[jj] > mag_max[jj]) 
                mag_max[jj] = mag_temp[jj];
            if(mag_temp[jj] < mag_min[jj]) 
                mag_min[jj] = mag_temp[jj];
        }
        usleep(12000);  // at 100 Hz ODR, new mag data is available every 10 ms
    }

    // Get hard iron correction
    mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
    mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
    mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts
    
    fMagnaticBias[0] = ( (float)mag_bias[0] ) * M_FACTOR * fMagneticAdjFactor[0];  // save mag biases in G for main program
    fMagnaticBias[1] = ( (float)mag_bias[1] ) * M_FACTOR * fMagneticAdjFactor[1];   
    fMagnaticBias[2] = ( (float)mag_bias[2] ) * M_FACTOR * fMagneticAdjFactor[2];  
       
    // Get soft iron correction estimate
    mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
    mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
    mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts

    avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
    avg_rad /= 3.0;

    fMagnaticScale[0] = avg_rad/((float)mag_scale[0]);
    fMagnaticScale[1] = avg_rad/((float)mag_scale[1]);
    fMagnaticScale[2] = avg_rad/((float)mag_scale[2]);
}
