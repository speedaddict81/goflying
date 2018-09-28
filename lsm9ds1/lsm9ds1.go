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

/*******Probably Don't Need These***************\
// EnableGyroBiasCal enables or disables motion bias compensation for the gyro.
// For flying we generally do not want this!
// func (mpu *MPU9250) EnableGyroBiasCal(enable bool) error {
// 	enableRegs := []byte{0xb8, 0xaa, 0xb3, 0x8d, 0xb4, 0x98, 0x0d, 0x35, 0x5d}
// 	disableRegs := []byte{0xb8, 0xaa, 0xaa, 0xaa, 0xb0, 0x88, 0xc3, 0xc5, 0xc7}

// 	if enable {
// 		if err := mpu.memWrite(CFG_MOTION_BIAS, &enableRegs); err != nil {
// 			return errors.New("Unable to enable motion bias compensation")
// 		}
// 	} else {
// 		if err := mpu.memWrite(CFG_MOTION_BIAS, &disableRegs); err != nil {
// 			return errors.New("Unable to disable motion bias compensation")
// 		}
// 	}

// 	return nil
// }

//******memwrite function********\\
// func (mpu *MPU9250) memWrite(addr uint16, data *[]byte) error {
// 	var err error
// 	var tmp = make([]byte, 2)

// 	tmp[0] = byte(addr >> 8)
// 	tmp[1] = byte(addr & 0xFF)

// 	// Check memory bank boundaries
// 	if tmp[1]+byte(len(*data)) > MPU_BANK_SIZE {
// 		return errors.New("Bad address: writing outside of memory bank boundaries")
// 	}

// 	err = mpu.i2cbus.WriteToReg(MPU_ADDRESS, MPUREG_BANK_SEL, tmp)
// 	if err != nil {
// 		return fmt.Errorf("MPU9250 Error selecting memory bank: %s\n", err)
// 	}

// 	err = mpu.i2cbus.WriteToReg(MPU_ADDRESS, MPUREG_MEM_R_W, *data)
// 	if err != nil {
// 		return fmt.Errorf("MPU9250 Error writing to the memory bank: %s\n", err)
// 	}

// 	return nil
// }


//**********Gyro and Accel Bias**********\\
// ReadAccelBias reads the bias accelerometer value stored on the chip.
// These values are set at the factory.
func (mpu *MPU9250) ReadAccelBias(sensitivityAccel int) error {
	a0x, err := mpu.i2cRead2(MPUREG_XA_OFFSET_H)
	if err != nil {
		return errors.New("MPU9250 Error: ReadAccelBias error reading chip")
	}
	a0y, err := mpu.i2cRead2(MPUREG_YA_OFFSET_H)
	if err != nil {
		return errors.New("MPU9250 Error: ReadAccelBias error reading chip")
	}
	a0z, err := mpu.i2cRead2(MPUREG_ZA_OFFSET_H)
	if err != nil {
		return errors.New("MPU9250 Error: ReadAccelBias error reading chip")
	}

	switch sensitivityAccel {
	case 16:
		mpu.a01 = float64(a0x >> 1)
		mpu.a02 = float64(a0y >> 1)
		mpu.a03 = float64(a0z >> 1)
	case 8:
		mpu.a01 = float64(a0x)
		mpu.a02 = float64(a0y)
		mpu.a03 = float64(a0z)
	case 4:
		mpu.a01 = float64(a0x << 1)
		mpu.a02 = float64(a0y << 1)
		mpu.a03 = float64(a0z << 1)
	case 2:
		mpu.a01 = float64(a0x << 2)
		mpu.a02 = float64(a0y << 2)
		mpu.a03 = float64(a0z << 2)
	default:
		return fmt.Errorf("MPU9250 Error: %d is not a valid acceleration sensitivity", sensitivityAccel)
	}

	log.Printf("MPU9250 Info: accel hardware bias read: %6f %6f %6f\n", mpu.a01, mpu.a02, mpu.a03)
	return nil
}

// ReadGyroBias reads the bias gyro value stored on the chip.
// These values are set at the factory.
func (mpu *MPU9250) ReadGyroBias(sensitivityGyro int) error {
	g0x, err := mpu.i2cRead2(MPUREG_XG_OFFS_USRH)
	if err != nil {
		return errors.New("MPU9250 Error: ReadGyroBias error reading chip")
	}
	g0y, err := mpu.i2cRead2(MPUREG_YG_OFFS_USRH)
	if err != nil {
		return errors.New("MPU9250 Error: ReadGyroBias error reading chip")
	}
	g0z, err := mpu.i2cRead2(MPUREG_ZG_OFFS_USRH)
	if err != nil {
		return errors.New("MPU9250 Error: ReadGyroBias error reading chip")
	}

	switch sensitivityGyro {
	case 2000:
		mpu.g01 = float64(g0x >> 1)
		mpu.g02 = float64(g0y >> 1)
		mpu.g03 = float64(g0z >> 1)
	case 1000:
		mpu.g01 = float64(g0x)
		mpu.g02 = float64(g0y)
		mpu.g03 = float64(g0z)
	case 500:
		mpu.g01 = float64(g0x << 1)
		mpu.g02 = float64(g0y << 1)
		mpu.g03 = float64(g0z << 1)
	case 250:
		mpu.g01 = float64(g0x << 2)
		mpu.g02 = float64(g0y << 2)
		mpu.g03 = float64(g0z << 2)
	default:
		return fmt.Errorf("MPU9250 Error: %d is not a valid gyro sensitivity", sensitivityGyro)
	}

	log.Printf("MPU9250 Info: Gyro hardware bias read: %6f %6f %6f\n", mpu.g01, mpu.g02, mpu.g03)
	return nil
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

