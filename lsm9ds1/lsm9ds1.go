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

	//TODO Determine if LSM has hidden access to LPFs
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

		//TODO LSM mag power up, if needed
		if err := mpu.i2cMagWrite(MPUREG_I2C_MST_CTRL, 0x40); err != nil {
			return nil, errors.New(fmt.Sprintf("Error setting up AK8963: %s", err))
		}
		//TODO Set mag sample rate to same as gyro/accel sample rate, up to max
		var magBits byte
		switch {
			case (lsm.sampleRate > 40):
				magBits = BITS_MAG_RATE_80
			case (rate > 20):
				magBits = BITS_MAG_RATE_40
			case (rate > 10):
				magBits = BITS_MAG_RATE_20
			default:
				magBits = BITS_MAG_RATE_10
		}
		//TODO Implement MagWrite fx
		if err := lsm.i2cMagWrite(CTRL_REG1_M, magBits); err != nil {
			return nil, errors.New(fmt.Sprintf("Error setting up LSM9DS1 magnetometer: %s", err))
		}
	

		time.Sleep(100 * time.Millisecond) // Make sure mag is ready
	}

	//TODO Determine necessity
	// Set clock source to PLL
	// if err := mpu.i2cWrite(MPUREG_PWR_MGMT_1, INV_CLK_PLL); err != nil {
	// 	return nil, errors.New(fmt.Sprintf("Error setting up LSM9DS1: %s", err))
	// }

	// Turn off all sensors -- Not sure if necessary, but it's in the InvenSense DMP driver
	// if err := mpu.i2cWrite(MPUREG_PWR_MGMT_2, 0x63); err != nil {
	// 	return nil, errors.New(fmt.Sprintf("Error setting up LSM9DS1: %s", err))
	// }
	// time.Sleep(100 * time.Millisecond)
	// // Turn on all gyro, all accel
	// if err := mpu.i2cWrite(MPUREG_PWR_MGMT_2, 0x00); err != nil {
	// 	return nil, errors.New(fmt.Sprintf("Error setting up LSM9DS1: %s", err))
	// }

	//LSM Accel/gyro Has no HW Offsets
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
	var gyroBits byte

	switch sensitivityGyro {
	case 2000:
		gyroBits = BITS_GYRO_2000
		lsm.scaleGyro = 2000.0 / float64(math.MaxInt16)
	// 1000 is not a valid DPS for LMS, default to 2000DPS
	case 1000:
		gyroBits = BITS_GYRO_2000
		lsm.scaleGyro = 2000.0 / float64(math.MaxInt16)
	case 500:
		gyroBits = BITS_GYRO_500
		lsm.scaleGyro = 500.0 / float64(math.MaxInt16)
	case 250:
		gyroBits = BITS_GYRO_250
		lsm.scaleGyro = 250.0 / float64(math.MaxInt16)
	default:
		//TODO Decide if 500 is an acceptable default, if so fix and remove return
		err = fmt.Errorf("LSM9DS1 Error: %d is not a valid gyro sensitivity", sensitivityGyro)
		return
	}

	//TODO Check Go syntax for proper use of bitwise OR, combine and check
	//		for acceptability prior to usage
	if errWrite := lsm.i2cWrite(CTRL_REG1_G, gyroBits | rateBits); errWrite != nil {
		err = errors.New("LSM9DS1 Error: couldn't set gyro sensitivity and sample rate")
	}

	return
}

// SetAccelSensitivity sets the accelerometer sensitivity of the LSM9DS1; it must be one of the following values:
// 2, 4, 8, 16, all in G (gravity).
func (lsm *LSM9DS1) SetAccelSensitivity(sensitivityAccel int) (err error) {
	var accelBits byte

	switch sensitivityAccel {
	case 16:
		accelBits = BITS_ACCEL_16G
		lsm.scaleAccel = 16.0 / float64(math.MaxInt16)
	case 8:
		accelBits = BITS_ACCEL_8G
		lsm.scaleAccel = 8.0 / float64(math.MaxInt16)
	case 4:
		accelBits = BITS_ACCEL_4G
		lsm.scaleAccel = 4.0 / float64(math.MaxInt16)
	case 2:
		accelBits = BITS_ACCEL_2G
		lsm.scaleAccel = 2.0 / float64(math.MaxInt16)
	default:
		err = fmt.Errorf("LSM9DS1 Error: %d is not a valid accel sensitivity", sensitivityAccel)
	}

	if errWrite := lsm.i2cWrite(CTRL_REG6_XL, accelBits); errWrite != nil {
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

//TODO WIP
func (lsm *LSM9DS1) readSensors() {
	var (
		g1, g2, g3, a1, a2, a3, m1, m2, m3, m4, tmp int16   // Current values
		avg1, avg2, avg3, ava1, ava2, ava3, avtmp   float64 // Accumulators for averages
		avm1, avm2, avm3                            int32
		n, nm                                       float64
		gaError, magError                           error
		t0, t, t0m, tm                              time.Time
		magSampleRate                               int
		curdata                                     *MPUData
	)

	//TODO MPU was little endian (Reference?), LSM is 2s complement, little endian? 7.26
	acRegMap := map[*int16]byte{
		&g1: LSM_OUT_X_L_G, &g2: LSM_OUT_Y_L_G, &g3: LSM_OUT_Z_L_G,
		&a1: LSM_OUT_X_L_XL, &a2: LSM_OUT_Y_L_XL, &a3: LSM_OUT_Z_L_XL,
		&tmp: LSM_OUT_TEMP_L,
	}
	//TODO LSM Register updates for magnetometer m4?
	// magRegMap := map[*int16]byte{
	// 	&m1: LSM_OUT_X_L_M, &m2: LSM_OUT_Y_L_M, &m3: LSM_OUT_Z_L_M, &m4: MPUREG_EXT_SENS_DATA_06,
	// }

	//TODO Determine acceptable Gyro/Accel/Mag rates for LSM
	if lsm.sampleRate >= 80 {
		magSampleRate = 80
	} else {
		magSampleRate = lsm.sampleRate
	}

	cC := make(chan *LSMData)
	defer close(cC)
	lsm.C = cC
	cAvg := make(chan *LSMData)
	defer close(cAvg)
	lsm.CAvg = cAvg
	cBuf := make(chan *LSMData, bufSize)
	defer close(cBuf)
	lsm.CBuf = cBuf
	lsm.cClose = make(chan bool)
	defer close(lsm.cClose)

	// TODO Run ticker at chosen sample rate, not ODR Rate
	//		fix formula to use lsm.sampleRate data type
	clock := time.NewTicker(time.Duration(int(1000.0/float32(lsm.sampleRate)+0.5)) * time.Millisecond)
	//TODO westphae: use the clock to record actual time instead of a timer
	defer clock.Stop()

	// TODO Tick at same rate as magnetometer, or slower
	clockMag := time.NewTicker(time.Duration(int(1000.0/float32(magSampleRate)+0.5)) * time.Millisecond)
	t0 = time.Now()
	t0m = time.Now()

	makeLSMData := func() *LSMData {
		d := LSMData{
			G1:      (float64(g1) - lsm.g01) * lsm.scaleGyro,
			G2:      (float64(g2) - lsm.g02) * lsm.scaleGyro,
			G3:      (float64(g3) - lsm.g03) * lsm.scaleGyro,
			A1:      (float64(a1) - lsm.a01) * lsm.scaleAccel,
			A2:      (float64(a2) - lsm.a02) * lsm.scaleAccel,
			A3:      (float64(a3) - lsm.a03) * lsm.scaleAccel,
			M1:      float64(m1) * lsm.mcal1,
			M2:      float64(m2) * lsm.mcal2,
			M3:      float64(m3) * lsm.mcal3,
			Temp:    float64(tmp)/340 + 36.53,
			GAError: gaError, MagError: magError,
			N: 1, 
			NM: 1,
			T: t, 
			TM: tm,
			DT: time.Duration(0), 
			DTM: time.Duration(0),
		}
		if gaError != nil {
			d.N = 0
		}
		if magError != nil {
			d.NM = 0
		}
		return &d
	}

	makeAvgLSMData := func() *LSMData {
		d := MPUData{}
		if n > 0.5 {
			d.G1 = (avg1/n - lsm.g01) * lsm.scaleGyro
			d.G2 = (avg2/n - lsm.g02) * lsm.scaleGyro
			d.G3 = (avg3/n - lsm.g03) * lsm.scaleGyro
			d.A1 = (ava1/n - lsm.a01) * lsm.scaleAccel
			d.A2 = (ava2/n - lsm.a02) * lsm.scaleAccel
			d.A3 = (ava3/n - lsm.a03) * lsm.scaleAccel
			d.Temp = (float64(avtmp)/n)/340 + 36.53
			d.N = int(n + 0.5)
			d.T = t
			d.DT = t.Sub(t0)
		} else {
			d.GAError = errors.New("LSM9DS1 Warning: No new accel/gyro values")
		}
		if nm > 0 {
			d.M1 = float64(avm1) * lsm.mcal1 / nm
			d.M2 = float64(avm2) * lsm.mcal2 / nm
			d.M3 = float64(avm3) * lsm.mcal3 / nm
			d.NM = int(nm + 0.5)
			d.TM = tm
			d.DTM = t.Sub(t0m)
		} else {
			d.MagError = errors.New("LSM9DS1 Warning: No new magnetometer values")
		}
		return &d
	}

	for {
		select {
		case t = <-clock.C: // Read accel/gyro data:
			for p, reg := range acRegMap {
				//TODO Replace with actual I2C readWord
				*p, gaError = lsm.i2cReadWord(LSM_GA_ADDRESS, reg)
				if gaError != nil {
					log.Println("LSM9DS1 Warning: error reading gyro/accel")
				}
			}
			curdata = makeLSMData()
			// Update accumulated values and increment count of gyro/accel readings
			avg1 += float64(g1)
			avg2 += float64(g2)
			avg3 += float64(g3)
			ava1 += float64(a1)
			ava2 += float64(a2)
			ava3 += float64(a3)
			avtmp += float64(tmp)
			avm1 += int32(m1)
			avm2 += int32(m2)
			avm3 += int32(m3)
			n++
			select {
			case cBuf <- curdata: // We update the buffer every time we read a new value.
			default: // If buffer is full, remove oldest value and put in newest.
				<-cBuf
				cBuf <- curdata
			}
		case tm = <-clockMag.C: // Read magnetometer data:
			//TODO Implement mag check for usage of m4 data in stratux/ahrs
			if lsm.enableMag {
				// Read the actual data
				for p, reg := range magRegMap {
					*p, magError = lsm.i2cReadWord(LSM_MAG_ADDRESS, reg)
					if magError != nil {
						log.Println("LSM9DS1 Warning: error reading magnetometer")
					}
				}

				//TODO Determine if LSM needs new data and overflow checks
				// Test validity of magnetometer data
				// if (byte(m1&0xFF)&AKM_DATA_READY) == 0x00 && (byte(m1&0xFF)&AKM_DATA_OVERRUN) != 0x00 {
				// 	log.Println("MPU9250 Warning: mag data not ready or overflow")
				// 	log.Printf("MPU9250 Warning: m1 LSB: %X\n", byte(m1&0xFF))
				// 	continue // Don't update the accumulated values
				// }

				// if (byte((m4>>8)&0xFF) & AKM_OVERFLOW) != 0x00 {
				// 	log.Println("MPU9250 Warning: mag data overflow")
				// 	log.Printf("MPU9250 Warning: m4 MSB: %X\n", byte((m1>>8)&0xFF))
				// 	continue // Don't update the accumulated values
				// }

				// Update values and increment count of magnetometer readings
				avm1 += int32(m1)
				avm2 += int32(m2)
				avm3 += int32(m3)
				nm++
			}
		case cC <- curdata: // Send the latest values
		case cAvg <- makeAvgLSMData(): // Send the averages
			avg1, avg2, avg3 = 0, 0, 0
			ava1, ava2, ava3 = 0, 0, 0
			avm1, avm2, avm3 = 0, 0, 0
			avtmp = 0
			n, nm = 0, 0
			t0, t0m = t, tm
		case <-lsm.cClose: // Stop the goroutine, ease up on the CPU
			break
		}
	}
}


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

func (lsm *LSM9DS1) i2cReadWord(i2cChan byte, register byte) (value int16, err error) {

	v, errWrite := lsm.i2cbus.ReadWordFromReg(i2cChan, register)
	if errWrite != nil {
		err = fmt.Errorf("LSM9DS1 Error reading %x: %s\n", register, err)
	} else {
		value = int16(v)
	}
	return
}

func (lsm *LSM9DS1) i2cWrite(register, value byte) (err error) {

	if errWrite := lsm.i2cbus.WriteByteToReg(LSM_GA_ADDRESS, register, value); errWrite != nil {
		err = fmt.Errorf("LSM9DS1 Error writing %X to %X: %s\n",
			value, register, errWrite)
	} else {
		time.Sleep(time.Millisecond)
	}
	return
}

// This is a separate function to prevent writing on the wrong I2C channel
func (lsm *LSM9DS1) i2cMagWrite(register, value byte) (err error) {

	if errWrite := lsm.i2cbus.WriteByteToReg(LSM_MAG_ADDRESS, register, value); errWrite != nil {
		err = fmt.Errorf("LSM9DS1 Error writing %X to %X: %s\n",
			value, register, errWrite)
	} else {
		time.Sleep(time.Millisecond)
	}
	return
}


