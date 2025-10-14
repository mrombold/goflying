// Package mpu9250 provides a synchronous, deterministic driver for the InvenSense MPU-9250.
package mpu9250

import (
	"errors"
	"time"

	"github.com/kidoman/embd"
)

type Sample struct {
	// Gyro in deg/s
	GxDPS, GyDPS, GzDPS float64
	// Accel in g
	AxG, AyG, AzG float64
	// Mag in microtesla (uT). Zeroed if mag disabled or not ready.
	MxUT, MyUT, MzUT float64

	IMUError error // gyro/accel read/config error (if any)
	MagError error // magnetometer read/config error (if any)
}

// Device is a synchronous MPU-9250 + AK8963 driver.
type Device struct {
	bus        embd.I2CBus
	addr       byte
	gyroScale  float64 // dps per LSB
	accelScale float64 // g per LSB
	magAdj     [3]float64
	magEnabled bool
}

type ODR uint16

const (
	ODR1000Hz ODR = 1000
	ODR500Hz  ODR = 500
	ODR200Hz  ODR = 200
	ODR100Hz  ODR = 100
	ODR50Hz   ODR = 50
	ODR25Hz   ODR = 25
)


// Gyro full-scale range (deg/s)
type GyroRange byte

const (
	Gyro250DPS  GyroRange = BITS_FS_250DPS
	Gyro500DPS  GyroRange = BITS_FS_500DPS
	Gyro1000DPS GyroRange = BITS_FS_1000DPS
	Gyro2000DPS GyroRange = BITS_FS_2000DPS
)

func (g GyroRange) scale() float64 {
	switch g {
	case Gyro250DPS:
		return 250.0 / 32768.0
	case Gyro500DPS:
		return 500.0 / 32768.0
	case Gyro1000DPS:
		return 1000.0 / 32768.0
	case Gyro2000DPS:
		return 2000.0 / 32768.0
	default:
		return 0 // invalid; caller should have validated
	}
}


func (o ODR) smplrtDiv() (byte, error) {
	base := 1000 // with DLPF enabled
	if o == 0 || int(base)%int(o) != 0 {
		return 0, errors.New("mpu9250: invalid ODR (not a divisor of 1000)")
	}
	div := base/int(o) - 1
	if div < 0 || div > 255 {
		return 0, errors.New("mpu9250: ODR out of SMPLRT_DIV range")
	}
	return byte(div), nil
}

// Accel full-scale range (g)
type AccelRange byte

const (
	Accel2G  AccelRange = BITS_FS_2G
	Accel4G  AccelRange = BITS_FS_4G
	Accel8G  AccelRange = BITS_FS_8G
	Accel16G AccelRange = BITS_FS_16G
)

func (a AccelRange) scale() float64 {
	switch a {
	case Accel2G:
		return 2.0 / 32768.0
	case Accel4G:
		return 4.0 / 32768.0
	case Accel8G:
		return 8.0 / 32768.0
	case Accel16G:
		return 16.0 / 32768.0
	default:
		return 0
	}
}

// Digital low-pass filter settings (datasheet-defined discrete options).
// These enums map 1:1 to the register’s DLPF_CFG bits.
type DLPF byte

const (
	DLPF_5HZ   DLPF = BITS_DLPF_CFG_5HZ
	DLPF_10HZ  DLPF = BITS_DLPF_CFG_10HZ
	DLPF_20HZ  DLPF = BITS_DLPF_CFG_20HZ
	DLPF_42HZ  DLPF = BITS_DLPF_CFG_42HZ
	DLPF_98HZ  DLPF = BITS_DLPF_CFG_98HZ
	DLPF_188HZ DLPF = BITS_DLPF_CFG_188HZ
)

// New sets up the device deterministically (no goroutines).
// All parameters are type-safe enums—no naked ints.
func New(
	bus embd.I2CBus,
	addr byte,
	gfs GyroRange,
	afs AccelRange,
	odr ODR,
	enableMag bool,
	gyroLPF DLPF,
	accelLPF DLPF,
) (*Device, error) {
	d := &Device{bus: bus, addr: addr, magEnabled: enableMag}

	// Reset, wake, select PLL clock
	if err := d.write(MPUREG_PWR_MGMT_1, BIT_H_RESET); err != nil { return nil, err }
	time.Sleep(100 * time.Millisecond)
	if err := d.write(MPUREG_PWR_MGMT_1, 0x00); err != nil { return nil, err }
	time.Sleep(10 * time.Millisecond)
	if err := d.write(MPUREG_PWR_MGMT_1, INV_CLK_PLL); err != nil { return nil, err }

	// Disable FIFO/interrupts: fully polled
	if err := d.write(MPUREG_FIFO_EN, 0x00); err != nil { return nil, err }
	if err := d.write(MPUREG_INT_ENABLE, 0x00); err != nil { return nil, err }

	// Ranges (sets scales)
	if err := d.SetGyroRange(gfs); err != nil { return nil, err }
	if err := d.SetAccelRange(afs); err != nil { return nil, err }

	// DLPFs first (ensures base rate is 1 kHz for SMPLRT_DIV math)
	if err := d.SetGyroLPF(gyroLPF); err != nil { return nil, err }
	if err := d.SetAccelLPF(accelLPF); err != nil { return nil, err }

	// ODR via SMPLRT_DIV (ties gyro/accel together in this driver)
	div, err := odr.smplrtDiv()
	if err != nil { return nil, err }
	if err := d.write(MPUREG_SMPLRT_DIV, div); err != nil { return nil, err }

	// Enable sensors
	if err := d.write(MPUREG_PWR_MGMT_2, 0x00); err != nil { return nil, err }

	// Magnetometer: bypass + factory adj + 16-bit continuous 100 Hz
	if d.magEnabled {
		if err := d.setupMagBypassAndCal(); err != nil {
			// Keep IMU alive even if mag fails
			d.magEnabled = false
		}
	}

	return d, nil
}


func (d *Device) Close() error {
	// Put to sleep (optional)
	_ = d.write(MPUREG_PWR_MGMT_1, 0x40) // sleep bit
	return nil
}

/* ------------ Public helpers (optional) ------------ */

// SetGyroLPF sets the gyro DLPF (type-safe).
func (d *Device) SetGyroLPF(lpf DLPF) error {
	switch lpf {
	case DLPF_5HZ, DLPF_10HZ, DLPF_20HZ, DLPF_42HZ, DLPF_98HZ, DLPF_188HZ:
		return d.write(MPUREG_CONFIG, byte(lpf))
	default:
		return errors.New("mpu9250: invalid gyro DLPF")
	}
}

// SetAccelLPF sets the accel DLPF (type-safe).
func (d *Device) SetAccelLPF(lpf DLPF) error {
	switch lpf {
	case DLPF_5HZ, DLPF_10HZ, DLPF_20HZ, DLPF_42HZ, DLPF_98HZ, DLPF_188HZ:
		return d.write(MPUREG_ACCEL_CONFIG_2, byte(lpf))
	default:
		return errors.New("mpu9250: invalid accel DLPF")
	}
}

// SetGyroRange sets the gyro full-scale range using the enum.
func (d *Device) SetGyroRange(r GyroRange) error {
	switch r {
	case Gyro250DPS, Gyro500DPS, Gyro1000DPS, Gyro2000DPS:
		// write the bits and set the scale
		if err := d.write(MPUREG_GYRO_CONFIG, byte(r)); err != nil {
			return err
		}
		d.gyroScale = r.scale()
		return nil
	default:
		return errors.New("mpu9250: invalid GyroRange")
	}
}

// SetAccelRange sets the accelerometer full-scale range using the enum.
func (d *Device) SetAccelRange(r AccelRange) error {
	switch r {
	case Accel2G, Accel4G, Accel8G, Accel16G:
		if err := d.write(MPUREG_ACCEL_CONFIG, byte(r)); err != nil {
			return err
		}
		d.accelScale = r.scale()
		return nil
	default:
		return errors.New("mpu9250: invalid AccelRange")
	}
}


/* ------------ Core read path (deterministic, blocking) ------------ */

// Read fetches one instantaneous sample (gyro, accel, and mag if enabled).
func (d *Device) Read() (Sample, error) {
	var s Sample

	// Read accel/gyro/temperature registers (6+6 words). We pull gyro first to minimize latency skew.
	gx, errG := d.readWord(MPUREG_GYRO_XOUT_H)
	gy, _   := d.readWord(MPUREG_GYRO_YOUT_H)
	gz, _   := d.readWord(MPUREG_GYRO_ZOUT_H)

	ax, errA := d.readWord(MPUREG_ACCEL_XOUT_H)
	ay, _    := d.readWord(MPUREG_ACCEL_YOUT_H)
	az, _    := d.readWord(MPUREG_ACCEL_ZOUT_H)

	if errG != nil || errA != nil {
		s.IMUError = firstErr(errG, errA)
	}

	s.GxDPS = float64(int16(gx)) * d.gyroScale
	s.GyDPS = float64(int16(gy)) * d.gyroScale
	s.GzDPS = float64(int16(gz)) * d.gyroScale

	s.AxG = float64(int16(ax)) * d.accelScale
	s.AyG = float64(int16(ay)) * d.accelScale
	s.AzG = float64(int16(az)) * d.accelScale

	// Magnetometer (optional)
	if d.magEnabled {
		mx, my, mz, errM := d.readMagOnce()
		if errM != nil {
			s.MagError = errM
		} else {
			// Convert raw counts to uT with factory adjustments
			s.MxUT = d.magAdj[0] * float64(mx)
			s.MyUT = d.magAdj[1] * float64(my)
			s.MzUT = d.magAdj[2] * float64(mz)
		}
	}

	return s, nil
}

/* ------------ Internals ------------ */
func (d *Device) setupMagBypassAndCal() error {
	// Local AK8963 constants (safe if your global constants differ/move)
	const (
		ak8963_CNTL1         = AK8963_CNTL1
		ak8963_ST1           = AK8963_ST1
		ak8963_HXL           = AK8963_HXL
		ak8963_ASAX          = AK8963_ASAX
		ak8963_ASAY          = AK8963_ASAY
		ak8963_ASAZ          = AK8963_ASAZ
		akm_16bit      byte  = 0x10
		akm_cont_100hz byte  = 0x06
		lsbToUT        float64 = 0.15 // 16-bit mode scale
	)

	// Enable bypass: disable AUX master, set BYPASS_EN
	uc, err := d.readByte(MPUREG_USER_CTRL); if err != nil { return err }
	if err := d.write(MPUREG_USER_CTRL, uc &^ BIT_AUX_IF_EN); err != nil { return err }
	time.Sleep(3 * time.Millisecond)
	if err := d.write(MPUREG_INT_PIN_CFG, BIT_BYPASS_EN); err != nil { return err }
	time.Sleep(3 * time.Millisecond)

	// Power down, go to fuse ROM to read ASA
	if err := d.write(ak8963_CNTL1, AKM_POWER_DOWN); err != nil { return err }
	time.Sleep(1 * time.Millisecond)
	if err := d.write(ak8963_CNTL1, AK8963_I2CDIS); err != nil { return err } // 0x0F: Fuse ROM access
	time.Sleep(1 * time.Millisecond)

	ax, err := d.readByte(ak8963_ASAX); if err != nil { return err }
	ay, err := d.readByte(ak8963_ASAY); if err != nil { return err }
	az, err := d.readByte(ak8963_ASAZ); if err != nil { return err }

	d.magAdj[0] = (float64(int(ax)-128)/256.0 + 1.0) * lsbToUT
	d.magAdj[1] = (float64(int(ay)-128)/256.0 + 1.0) * lsbToUT
	d.magAdj[2] = (float64(int(az)-128)/256.0 + 1.0) * lsbToUT

	// Exit fuse ROM; set 16-bit, continuous 100 Hz
	if err := d.write(ak8963_CNTL1, AKM_POWER_DOWN); err != nil { return err }
	time.Sleep(1 * time.Millisecond)
	if err := d.write(ak8963_CNTL1, akm_16bit|akm_cont_100hz); err != nil { return err }
	time.Sleep(10 * time.Millisecond)
	return nil
}


// readMagOnce reads AK8963 in bypass mode; returns raw int16 (little-endian).
func (d *Device) readMagOnce() (mx, my, mz int16, err error) {
	// Check DRDY
	st1, e := d.readByte(AK8963_ST1)
	if e != nil { return 0,0,0,e }
	if st1 & AKM_DATA_READY == 0 { return 0,0,0,errors.New("mag not ready") }

	// Read 7 bytes: HXL..HZH, ST2
	buf := make([]byte, 7)
	if e = d.readBlock(AK8963_HXL, buf); e != nil { return 0,0,0,e }

	// Overflow check
	if buf[6] & AKM_OVERFLOW != 0 { return 0,0,0,errors.New("mag overflow") }

	// Little endian
	mx = int16(uint16(buf[1])<<8 | uint16(buf[0]))
	my = int16(uint16(buf[3])<<8 | uint16(buf[2]))
	mz = int16(uint16(buf[5])<<8 | uint16(buf[4]))
	return
}

/* ------------ I2C helpers ------------ */

func (d *Device) write(reg, val byte) error {
	return d.bus.WriteByteToReg(d.addr, reg, val)
}

func (d *Device) readByte(reg byte) (byte, error) {
	return d.bus.ReadByteFromReg(d.addr, reg)
}

func (d *Device) readWord(reg byte) (uint16, error) {
	return d.bus.ReadWordFromReg(d.addr, reg)
}

func (d *Device) readBlock(reg byte, dst []byte) error {
	return d.bus.ReadFromReg(d.addr, reg, dst)
}

func firstErr(a, b error) error {
	if a != nil { return a }
	return b
}
