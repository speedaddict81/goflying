package lsm9ds1

// Approach adapted from the InvenSense DMP 6.1 drivers
// Also referenced https://github.com/brianc118/MPU9250/blob/master/MPU9250.cpp

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

//*******Probably Don't Need These***************\\
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