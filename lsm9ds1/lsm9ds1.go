package lsm9ds1


import (
	"errors"
	"fmt"
	"log"
	"math"
	"time"

	"../embd"
	_ "../embd/host/all"
	_ "../embd/host/rpi"
)

const (
	bufSize  = 250 // Size of buffer storing instantaneous sensor values
	scaleMag = 9830.0 / 65536
)

// LSMData contains all the values measured by an LSM9DS1.
type LSMData struct {
	G1, G2, G3        float64
	A1, A2, A3        float64
	M1, M2, M3        float64
	Temp              float64
	GAError, MagError error
	N, NM             int
	T, TM             time.Time
	DT, DTM           time.Duration
}

/*
LSM9DS1 represents an Ozzmaker BerryIMU 2 w/LSM9DS1 9DoF chip.
All communication is via channels.
*/
type LSM9DS1 struct {
	i2cbus                embd.I2CBus
	scaleGyro, scaleAccel float64         // Max sensor reading for value 2**15-1
	sampleRate            int             // Sample rate for sensor readings, Hz
	enableMag             bool            // Read the magnetometer?
	mcal1, mcal2, mcal3   float64         // Hardware magnetometer calibration values, uT
	a01, a02, a03         float64         // Hardware accelerometer calibration values, G
	g01, g02, g03         float64         // Hardware gyro calibration values, °/s
	C                     <-chan *LSMData // Current instantaneous sensor values
	CAvg                  <-chan *LSMData // Average sensor values (since CAvg last read)
	CBuf                  <-chan *LSMData // Buffer of instantaneous sensor values
	cClose                chan bool       // Turn off MPU polling
}

/*
NewLSM9DS1 creates a new LSM9DS1 object according to the supplied parameters.  If there is no LSM9DS1 available or there
is an error creating the object, an error is returned.
*/
func NewLSM9DS1(sensitivityGyro, sensitivityAccel, sampleRate int, enableMag bool, applyHWOffsets bool) (*LSM9DS1, error) {
	var lsm = new(LSM9DS1)

	lsm.sampleRate = sampleRate
	//lsm.enableMag = enableMag
	lsm.enableMag = false

	lsm.i2cbus = embd.NewI2CBus(1)

	//*****Chip Initialization******\\
	//TODO Use LSM Chip initialization as needed (set ODR, sleep mode setting, etc)

	// TODO Determine if needed (Reset device defaults)
	// if err := lsm.i2cWrite(????, BIT_H_RESET); err != nil {
	// 	return nil, errors.New(fmt.Sprintf("Error resetting LSM9DS1: %s", err))
	// }

	//TODO Determine if needed (Wake up chip)
	// time.Sleep(100 * time.Millisecond)
	// if err := mpu.i2cWrite(MPUREG_PWR_MGMT_1, 0x00); err != nil {
	// 	return nil, errors.New(fmt.Sprintf("Error waking LSM9DS1: %s", err))
	// }

	// Set Gyro and Accel sensitivities
	if err := lsm.SetupGyroAndAccel(sampleRate, sensitivityGyro, sensitivityAccel); err != nil {
		return nil, errors.New(fmt.Sprintf("Error setting up LSM9DS1 gyro/accel: %s", err))
	}

	//TODO WIP Determine LSM-MPU differences for LPFs
	// sampRate := byte(1000/mpu.sampleRate - 1)
	// Default: Set Gyro LPF to half of sample rate
	// if err := mpu.SetGyroLPF(sampRate >> 1); err != nil {
	// 	return nil, errors.New(fmt.Sprintf("Error setting LSM9DS1 Gyro LPF: %s", err))
	// }

	// Default: Set Accel LPF to half of sample rate
	// if err := mpu.SetAccelLPF(sampRate >> 1); err != nil {
	// 	return nil, errors.New(fmt.Sprintf("Error setting LSM9DS1 Accel LPF: %s", err))
	// }


	// TODO Turn off FIFO buffer
	// For LSM should be no issue leaving on
	// if err := mpu.i2cWrite(MPUREG_FIFO_EN, 0x00); err != nil {
	// 	return nil, errors.New(fmt.Sprintf("LSM9DS1 Error: couldn't disable FIFO: %s", err))
	// }

	// TODO Turn off interrupts
	// LSM Off by default
	// if err := mpu.i2cWrite(MPUREG_INT_ENABLE, 0x00); err != nil {
	// 	return nil, errors.New(fmt.Sprintf("LSM9DS1 Error: couldn't disable interrupts: %s", err))
	// }

	// TODO Set up magnetometer
	// The LSM will not use the slave model used in the MPU chip
	// We will communicate directly using the correct I2C channel
	// and Mag registers
	if lsm.enableMag {
		if err := mpu.ReadMagCalibration(); err != nil {
			return nil, errors.New(fmt.Sprintf("Error reading calibration from magnetometer: %s", err))
		}

		// Set up AK8963 master mode, master clock and ES bit
		if err := mpu.i2cWrite(MPUREG_I2C_MST_CTRL, 0x40); err != nil {
			return nil, errors.New(fmt.Sprintf("Error setting up AK8963: %s", err))
		}
		// Slave 0 reads from AK8963
		if err := mpu.i2cWrite(MPUREG_I2C_SLV0_ADDR, BIT_I2C_READ|AK8963_I2C_ADDR); err != nil {
			return nil, errors.New(fmt.Sprintf("Error setting up AK8963: %s", err))
		}
		// Compass reads start at this register
		if err := mpu.i2cWrite(MPUREG_I2C_SLV0_REG, AK8963_ST1); err != nil {
			return nil, errors.New(fmt.Sprintf("Error setting up AK8963: %s", err))
		}
		// Enable 8-byte reads on slave 0
		if err := mpu.i2cWrite(MPUREG_I2C_SLV0_CTRL, BIT_SLAVE_EN|8); err != nil {
			return nil, errors.New(fmt.Sprintf("Error setting up AK8963: %s", err))
		}
		// Slave 1 can change AK8963 measurement mode
		if err := mpu.i2cWrite(MPUREG_I2C_SLV1_ADDR, AK8963_I2C_ADDR); err != nil {
			return nil, errors.New(fmt.Sprintf("Error setting up AK8963: %s", err))
		}
		if err := mpu.i2cWrite(MPUREG_I2C_SLV1_REG, AK8963_CNTL1); err != nil {
			return nil, errors.New(fmt.Sprintf("Error setting up AK8963: %s", err))
		}
		// Enable 1-byte reads on slave 1
		if err := mpu.i2cWrite(MPUREG_I2C_SLV1_CTRL, BIT_SLAVE_EN|1); err != nil {
			return nil, errors.New(fmt.Sprintf("Error setting up AK8963: %s", err))
		}
		// Set slave 1 data
		if err := mpu.i2cWrite(MPUREG_I2C_SLV1_DO, AKM_SINGLE_MEASUREMENT); err != nil {
			return nil, errors.New(fmt.Sprintf("Error setting up AK8963: %s", err))
		}
		// Triggers slave 0 and 1 actions at each sample
		if err := mpu.i2cWrite(MPUREG_I2C_MST_DELAY_CTRL, 0x03); err != nil {
			return nil, errors.New(fmt.Sprintf("Error setting up AK8963: %s", err))
		}

		// Set AK8963 sample rate to same as gyro/accel sample rate, up to max
		var ak8963Rate byte
		if mpu.sampleRate < AK8963_MAX_SAMPLE_RATE {
			ak8963Rate = 0
		} else {
			ak8963Rate = byte(mpu.sampleRate/AK8963_MAX_SAMPLE_RATE - 1)
		}

		// Not so sure of this one--I2C Slave 4??!
		if err := mpu.i2cWrite(MPUREG_I2C_SLV4_CTRL, ak8963Rate); err != nil {
			return nil, errors.New(fmt.Sprintf("Error setting up AK8963: %s", err))
		}

		time.Sleep(100 * time.Millisecond) // Make sure mag is ready
	}

	//TODO Determine necessity
	// Set clock source to PLL
	if err := mpu.i2cWrite(MPUREG_PWR_MGMT_1, INV_CLK_PLL); err != nil {
		return nil, errors.New(fmt.Sprintf("Error setting up LSM9DS1: %s", err))
	}

	// Turn off all sensors -- Not sure if necessary, but it's in the InvenSense DMP driver
	// if err := mpu.i2cWrite(MPUREG_PWR_MGMT_2, 0x63); err != nil {
	// 	return nil, errors.New(fmt.Sprintf("Error setting up LSM9DS1: %s", err))
	// }
	// time.Sleep(100 * time.Millisecond)
	// // Turn on all gyro, all accel
	// if err := mpu.i2cWrite(MPUREG_PWR_MGMT_2, 0x00); err != nil {
	// 	return nil, errors.New(fmt.Sprintf("Error setting up LSM9DS1: %s", err))
	// }

	//LSM Has no HW Offsets
	// if applyHWOffsets {
	// 	if err := mpu.ReadAccelBias(sensitivityAccel); err != nil {
	// 		return nil, err
	// 	}
	// 	if err := mpu.ReadGyroBias(sensitivityGyro); err != nil {
	// 		return nil, err
	// 	}
	// }

	// Usually we don't want the automatic gyro bias compensation - it pollutes the gyro in a non-inertial frame.
	// LSM No gyro bias
	// if err := mpu.EnableGyroBiasCal(false); err != nil {
	// 	return nil, err
	// }

	go mpu.readSensors()

	// Give the IMU time to fully initialize and then clear out any bad values from the averages.
	time.Sleep(500 * time.Millisecond) // Make sure it's ready
	<-lsm.CAvg

	return lsm, nil
}

//TODO WIP
// SetupGyroAccel sets the ODR (sample) rate, Accel, and Gyro Sensistivities
func (lsm *LSM9DS1) SetupGyroAndAccel(rate int, sensitivityGyro int, sensitivityAccel int) (err error) {
	var r byte
	var lowPower bool = false
	// BITS_ODR_RATE_15		  = 0x20
	// BITS_ODR_RATE_59		  = 0x40
	// BITS_ODR_RATE_119		  = 0x60
	// BITS_ODR_RATE_238		  = 0x80

	//TODO Decide if we should set lsm.sampleRate to actual rate used so LPF can be set
	// Stratux uses sample rate of 50Hz and LPF of 20 or 21 Hz 
	switch {
	case (rate >= 120):
		r = BITS_ODR_RATE_238
	case (rate >= 60):
		r = BITS_ODR_RATE_119
		lowPower = true
	case (rate > 15):
		r = BITS_ODR_RATE_59
		lowPower = true
	default:
		r = BITS_ODR_RATE_15
		lowPower = true
	}

	if err := lsm.SetGyroSensitivity(sensitivityGyro, r); err != nil {
		return
	}

	//set accelerometer sensitivity
	if err := lsm.SetAccelSensitivity(sensitivityAccel); err != nil {
		return
	}

	if err := lsm.SetPowerLevel(lowPower); err != nil {
		return
	}

	return
}

// SetGyroSensitivity sets the gyro sensitivity of the LSM9DS1; it must be one of the following values:
// 250, 500, 2000 (all in deg/s).
func (lsm *LSM9DS1) SetGyroSensitivity(sensitivityGyro int, rateBits byte) (err error) {
	var sensGyro byte

	switch sensitivityGyro {
	case 2000:
		sensGyro = BITS_GYRO_2000
		lsm.scaleGyro = 2000.0 / float64(math.MaxInt16)
	// 1000 is not a valid DPS for LMS, default to 2000DPS
	case 1000:
		sensGyro = BITS_GYRO_2000
		lsm.scaleGyro = 2000.0 / float64(math.MaxInt16)
	case 500:
		sensGyro = BITS_GYRO_500
		lsm.scaleGyro = 500.0 / float64(math.MaxInt16)
	case 250:
		sensGyro = BITS_GYRO_250
		lsm.scaleGyro = 250.0 / float64(math.MaxInt16)
	default:
		//TODO Decide if 500 is an acceptable default, if so fix and remove return
		err = fmt.Errorf("LSM9DS1 Error: %d is not a valid gyro sensitivity", sensitivityGyro)
		return
	}

	//TODO Check Go syntax for proper use of bitwise AND
	if errWrite := lsm.i2cWrite(CTRL_REG1_G, sensGyro&rateBits); errWrite != nil {
		err = errors.New("LSM9DS1 Error: couldn't set gyro sensitivity and sample rate")
	}

	return
}

// SetAccelSensitivity sets the accelerometer sensitivity of the LSM9DS1; it must be one of the following values:
// 2, 4, 8, 16, all in G (gravity).
func (lsm *LSM9DS1) SetAccelSensitivity(sensitivityAccel int) (err error) {
	var sensAccel byte

	switch sensitivityAccel {
	case 16:
		sensAccel = BITS_ACCEL_16G
		lsm.scaleAccel = 16.0 / float64(math.MaxInt16)
	case 8:
		sensAccel = BITS_ACCEL_8G
		lsm.scaleAccel = 8.0 / float64(math.MaxInt16)
	case 4:
		sensAccel = BITS_ACCEL_4G
		lsm.scaleAccel = 4.0 / float64(math.MaxInt16)
	case 2:
		sensAccel = BITS_ACCEL_2G
		lsm.scaleAccel = 2.0 / float64(math.MaxInt16)
	default:
		err = fmt.Errorf("LSM9DS1 Error: %d is not a valid accel sensitivity", sensitivityAccel)
	}

	if errWrite := lsm.i2cWrite(CTRL_REG6_XL, sensAccel); errWrite != nil {
		err = errors.New("LSM9DS1 Error: couldn't set accel sensitivity")
	}

	return
}

func (lsm *LSM9DS1) SetPowerLevel(lowPow bool) (err error) {
	// CTRL_REG3_G				  = 0x12
	// BITS_LP_MODE_EN			  = 0x80
	// BITS_LP_MODE_DIS		  = 0x00
	var pm byte
	if lowPow{
		pm = BITS_LP_MODE_EN
	}
	else {
		pm = BITS_LP_MODE_DIS
	}
	if errWrite := lsm.i2cWrite(CTRL_REG3_G, pm); errWrite != nil {
		err = errors.New("LSM9DS1 Error: couldn't set power level")
	}

	return
}

//TODO LSM doesn't have mutable LPF that I can find
// SetGyroLPF sets the low pass filter for the gyro.
// func (lsm *LSM9DS1) SetGyroLPF(rate byte) (err error) {
// 	var r byte
// 	switch {
// 	case rate >= 188:
// 		r = BITS_DLPF_CFG_188HZ
// 	case rate >= 98:
// 		r = BITS_DLPF_CFG_98HZ
// 	case rate >= 42:
// 		r = BITS_DLPF_CFG_42HZ
// 	case rate >= 20:
// 		r = BITS_DLPF_CFG_20HZ
// 	case rate >= 10:
// 		r = BITS_DLPF_CFG_10HZ
// 	default:
// 		r = BITS_DLPF_CFG_5HZ
// 	}

// 	errWrite := mpu.i2cWrite(MPUREG_CONFIG, r)
// 	if errWrite != nil {
// 		err = fmt.Errorf("MPU9250 Error: couldn't set Gyro LPF: %s", errWrite)
// 	}
// 	return
// }

//TODO LSM has no user-defined LPF
// SetAccelLPF sets the low pass filter for the accelerometer.
// func (lsm *LSM9DS1) SetAccelLPF(rate byte) (err error) {
// 	var r byte
// 	switch {
// 	case rate >= 218:
// 		r = BITS_DLPF_CFG_188HZ
// 	case rate >= 99:
// 		r = BITS_DLPF_CFG_98HZ
// 	case rate >= 45:
// 		r = BITS_DLPF_CFG_42HZ
// 	case rate >= 21:
// 		r = BITS_DLPF_CFG_20HZ
// 	case rate >= 10:
// 		r = BITS_DLPF_CFG_10HZ
// 	default:
// 		r = BITS_DLPF_CFG_5HZ
// 	}

// 	errWrite := mpu.i2cWrite(MPUREG_ACCEL_CONFIG_2, r)
// 	if errWrite != nil {
// 		err = fmt.Errorf("LSM9DS1 Error: couldn't set Accel LPF: %s", errWrite)
// 	}
// 	return
// }


/*******Probably Don't Need These***************\
// EnableGyroBiasCal enables or disables motion bias compensation for the gyro.
// For flying we generally do not want this!
// func (mpu *MPU9250) EnableGyroBiasCal(enable bool) error {
// }

//******memwrite function********\\
// func (mpu *MPU9250) memWrite(addr uint16, data *[]byte) error {
// }


//**********Gyro and Accel Bias**********\\
// ReadAccelBias reads the bias accelerometer value stored on the chip.
// These values are set at the factory.
func (mpu *MPU9250) ReadAccelBias(sensitivityAccel int) error {
}

// ReadGyroBias reads the bias gyro value stored on the chip.
// These values are set at the factory.
func (mpu *MPU9250) ReadGyroBias(sensitivityGyro int) error {
}
*/

//TODO: We'll prefer to use the bare ReadWord function
func (lsm *LSM9DS1) i2cRead2(register byte) (value int16, err error) {

	v, errWrite := lsm.i2cbus.ReadWordFromReg(LSM_ADDRESS, register)
	if errWrite != nil {
		err = fmt.Errorf("LSM9DS1 Error reading %x: %s\n", register, err)
	} else {
		value = int16(v)
	}
	return
}

