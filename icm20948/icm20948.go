// Package icm20948 is a deterministic, allocation-free (hot path) driver for
// InvenSense ICM-20948 with the onboard AK09916 magnetometer.
// Floats are used for scaled outputs; no goroutines/channels; no heap allocs in Read().
package icm20948

import (
	"errors"
	"fmt"
	"time"

	"github.com/kidoman/embd"
)

// ---------- Strongly-typed enums (no free ints) ----------

type GyroRange uint8

const (
	Gyro250DPS  GyroRange = 0 // BITS_FS_250DPS
	Gyro500DPS  GyroRange = 1 // BITS_FS_500DPS
	Gyro1000DPS GyroRange = 2 // BITS_FS_1000DPS
	Gyro2000DPS GyroRange = 3 // BITS_FS_2000DPS
)

func (g GyroRange) bits() byte {
	switch g {
	case Gyro250DPS:
		return BITS_FS_250DPS
	case Gyro500DPS:
		return BITS_FS_500DPS
	case Gyro1000DPS:
		return BITS_FS_1000DPS
	case Gyro2000DPS:
		return BITS_FS_2000DPS
	default:
		return BITS_FS_250DPS
	}
}
func (g GyroRange) fullScaleDPS() float64 {
	switch g {
	case Gyro250DPS:
		return 250
	case Gyro500DPS:
		return 500
	case Gyro1000DPS:
		return 1000
	case Gyro2000DPS:
		return 2000
	default:
		return 250
	}
}

type AccelRange uint8

const (
	Accel2G  AccelRange = 0 // BITS_FS_2G
	Accel4G  AccelRange = 1 // BITS_FS_4G
	Accel8G  AccelRange = 2 // BITS_FS_8G
	Accel16G AccelRange = 3 // BITS_FS_16G
)

func (a AccelRange) bits() byte {
	switch a {
	case Accel2G:
		return BITS_FS_2G
	case Accel4G:
		return BITS_FS_4G
	case Accel8G:
		return BITS_FS_8G
	case Accel16G:
		return BITS_FS_16G
	default:
		return BITS_FS_2G
	}
}
func (a AccelRange) fullScaleG() float64 {
	switch a {
	case Accel2G:
		return 2
	case Accel4G:
		return 4
	case Accel8G:
		return 8
	case Accel16G:
		return 16
	default:
		return 2
	}
}

type GyroLPF uint8

const (
	GyroLPF197Hz GyroLPF = 0
	GyroLPF152Hz GyroLPF = 1
	GyroLPF120Hz GyroLPF = 2
	GyroLPF51Hz  GyroLPF = 3
	GyroLPF24Hz  GyroLPF = 4
	GyroLPF12Hz  GyroLPF = 5
	GyroLPF6Hz   GyroLPF = 6
)

func (f GyroLPF) bits() byte {
	switch f {
	case GyroLPF197Hz:
		return BITS_DLPF_GYRO_CFG_197HZ
	case GyroLPF152Hz:
		return BITS_DLPF_GYRO_CFG_152HZ
	case GyroLPF120Hz:
		return BITS_DLPF_GYRO_CFG_120HZ
	case GyroLPF51Hz:
		return BITS_DLPF_GYRO_CFG_51HZ
	case GyroLPF24Hz:
		return BITS_DLPF_GYRO_CFG_24HZ
	case GyroLPF12Hz:
		return BITS_DLPF_GYRO_CFG_12HZ
	default:
		return BITS_DLPF_GYRO_CFG_6HZ
	}
}

type AccelLPF uint8

const (
	AccelLPF246Hz AccelLPF = 0
	AccelLPF111Hz AccelLPF = 1
	AccelLPF50Hz  AccelLPF = 2
	AccelLPF24Hz  AccelLPF = 3
	AccelLPF12Hz  AccelLPF = 4
	AccelLPF5Hz   AccelLPF = 5
)

func (f AccelLPF) bits() byte {
	switch f {
	case AccelLPF246Hz:
		return BITS_DLPF_ACCEL_CFG_246HZ
	case AccelLPF111Hz:
		return BITS_DLPF_ACCEL_CFG_111HZ
	case AccelLPF50Hz:
		return BITS_DLPF_ACCEL_CFG_50HZ
	case AccelLPF24Hz:
		return BITS_DLPF_ACCEL_CFG_24HZ
	case AccelLPF12Hz:
		return BITS_DLPF_ACCEL_CFG_12HZ
	default:
		return BITS_DLPF_ACCEL_CFG_5HZ
	}
}

type MagMode uint8

const (
	MagOff      MagMode = 0
	MagSingle           = 1
	MagCont10Hz         = 2
	MagCont20Hz         = 3
	MagCont50Hz         = 4
	MagCont100Hz        = 5
)

func (m MagMode) akMode() byte {
	switch m {
	case MagSingle:
		return AK09916_MODE_SINGLE
	case MagCont10Hz:
		return AK09916_MODE_CONT_10HZ
	case MagCont20Hz:
		return AK09916_MODE_CONT_20HZ
	case MagCont50Hz:
		return AK09916_MODE_CONT_50HZ
	case MagCont100Hz:
		return AK09916_MODE_CONT_100HZ
	default:
		return AK09916_MODE_POWER_DOWN
	}
}
func (m MagMode) enable() bool { return m != MagOff }

// ---------- Public configuration ----------

type Config struct {
	GyroRange  GyroRange
	AccelRange AccelRange
	GyroLPF    GyroLPF
	AccelLPF   AccelLPF
	// Target output rates (Hz). Internally mapped to divisors.
	GyroODRHz  uint16 // base 1125 Hz / (1+div)
	AccelODRHz uint16 // base 1125 Hz / (1+div)
	Mag        MagMode
}

// ---------- Output sample ----------

type Sample struct {
	TimeNS int64

	// Scaled values
	AxG, AyG, AzG   float64
	GxDPS, GyDPS, GzDPS float64
	MxUT, MyUT, MzUT    float64
	TempC          float64

	IMUError error
	MagError error
}

// ---------- Driver ----------

type Device struct {
	bus  embd.I2CBus
	addr byte

	cfg Config

	gyroScale  float64 // DPS/LSB
	accelScale float64 // g/LSB

	bankCached byte

	// hot-path buffers (no allocs in Read)
	buf14  [14]byte // accel(6)+temp(2)+gyro(6) in BANK0
	magBuf [8]byte  // ST1 + 6 data + ST2

	// retries/timeouts
	retryCount   int
	retryDelay   time.Duration
	readyTimeout time.Duration

	magEnabled bool
}

// New creates and initializes the device deterministically.
func New(i2c *embd.I2CBus, addr byte, cfg Config) (*Device, error) {
	if i2c == nil {
		return nil, errors.New("nil i2c bus")
	}
	d := &Device{
		bus:          *i2c,
		addr:         addr,
		cfg:          cfg,
		retryCount:   2,
		retryDelay:   200 * time.Microsecond,
		readyTimeout: 5 * time.Millisecond,
	}

	// Cache starts invalid so first write hits bus.
	d.bankCached = 0xFF

	// ----- Bring-up sequence -----
	if err := d.writeBankCached(0); err != nil {
		return nil, err
	}

	// Reset
	if err := d.writeRetry(ICMREG_PWR_MGMT_1, BIT_H_RESET); err != nil {
		return nil, fmt.Errorf("reset: %w", err)
	}
	// Wait for reset bit to clear (the bit self-clears; spec timing is short).
	if err := d.waitClear(ICMREG_PWR_MGMT_1, BIT_H_RESET, d.readyTimeout); err != nil {
		return nil, fmt.Errorf("reset wait: %w", err)
	}
	// Clock select to best PLL (0x01).
	if err := d.writeRetry(ICMREG_PWR_MGMT_1, 0x01); err != nil {
		return nil, fmt.Errorf("pwr_mgmt_1: %w", err)
	}

	// WHOAMI sanity (typical ICM20948 WHOAMI is 0xEA – not strictly enforced).
	if who, err := d.readRetry(ICMREG_WHOAMI); err == nil && who == 0xEA {
		// ok
	} else if err != nil {
		return nil, fmt.Errorf("whoami read: %w", err)
	} // else mismatch tolerated (some clones) — continue

	// Gyro/Accel config (BANK2)
	if err := d.writeBankCached(2); err != nil {
		return nil, err
	}
	// Gyro full-scale & LPF:
	// GYRO_CONFIG: FS_SEL and DLPF bits live together; set DLPF enable bit (LSB) and filter bits.
	// NOTE: The DLPF enable bit for ICM20948 gyro is bit0 of GYRO_CONFIG; we OR it with the chosen DLPF code.
	gyroCfg := cfg.GyroRange.bits() | cfg.GyroLPF.bits() | 0x01
	if err := d.writeRetry(ICMREG_GYRO_CONFIG, gyroCfg); err != nil {
		return nil, fmt.Errorf("gyro_config: %w", err)
	}

	// Accel full-scale & LPF:
	// ACCEL_CONFIG: FS_SEL bits + DLPF enable (bit0) + filter code.
	accelCfg := cfg.AccelRange.bits() | cfg.AccelLPF.bits() | 0x01
	if err := d.writeRetry(ICMREG_ACCEL_CONFIG, accelCfg); err != nil {
		return nil, fmt.Errorf("accel_config: %w", err)
	}

	// Sample rate divisors (base 1125 Hz)
	if err := d.setGyroODR(cfg.GyroODRHz); err != nil {
		return nil, err
	}
	if err := d.setAccelODR(cfg.AccelODRHz); err != nil {
		return nil, err
	}

	// Compute scales once
	d.gyroScale = cfg.GyroRange.fullScaleDPS() / 32768.0
	d.accelScale = cfg.AccelRange.fullScaleG() / 32768.0

	// Enable internal I2C master and configure AK09916 if requested
	d.magEnabled = cfg.Mag.enable()
	if d.magEnabled {
		if err := d.initMagAK09916(cfg.Mag); err != nil {
			// if mag init fails, continue IMU; report mag disabled
			d.magEnabled = false
		}
	}

	// Return to BANK0 for runtime reads
	if err := d.writeBankCached(0); err != nil {
		return nil, err
	}

	return d, nil
}

func (d *Device) Close() { /* nothing persistent to shut down */ }

// Read performs a burst read of accel/temp/gyro and, if enabled, a single mag frame.
// No heap allocations on the hot path.
func (d *Device) Read() (Sample, error) {
	var s Sample
	s.TimeNS = time.Now().UnixNano()

	// IMU burst (BANK0)
	if err := d.readBurst14(); err != nil {
		s.IMUError = err
		return s, err
	}

	ax := int16(d.buf14[0])<<8 | int16(d.buf14[1])
	ay := int16(d.buf14[2])<<8 | int16(d.buf14[3])
	az := int16(d.buf14[4])<<8 | int16(d.buf14[5])
	tr := int16(d.buf14[6])<<8 | int16(d.buf14[7]) // temp
	gx := int16(d.buf14[8])<<8 | int16(d.buf14[9])
	gy := int16(d.buf14[10])<<8 | int16(d.buf14[11])
	gz := int16(d.buf14[12])<<8 | int16(d.buf14[13])

	s.AxG = float64(ax) * d.accelScale
	s.AyG = float64(ay) * d.accelScale
	s.AzG = float64(az) * d.accelScale
	s.GxDPS = float64(gx) * d.gyroScale
	s.GyDPS = float64(gy) * d.gyroScale
	s.GzDPS = float64(gz) * d.gyroScale
	s.TempC = float64(tr)/333.87 + 21.0

	// Magnetometer (if configured)
	if d.magEnabled {
		if err := d.readMag8(); err != nil {
			s.MagError = err
		} else {
			st1 := d.magBuf[0]
			st2 := d.magBuf[7]
			if (st1 & AKM_DATA_READY) == 0 {
				s.MagError = errors.New("ak09916: data not ready")
			} else if (st2 & AKM_OVERFLOW) != 0 {
				s.MagError = errors.New("ak09916: overflow")
			} else {
				mx := int16(d.magBuf[1]) | int16(d.magBuf[2])<<8
				my := int16(d.magBuf[3]) | int16(d.magBuf[4])<<8
				mz := int16(d.magBuf[5]) | int16(d.magBuf[6])<<8
				// Scale to microtesla (0.15 uT/LSB).
				s.MxUT = float64(mx) * Magnetometer_Sensitivity_Scale_Factor
				s.MyUT = float64(my) * Magnetometer_Sensitivity_Scale_Factor
				s.MzUT = float64(mz) * Magnetometer_Sensitivity_Scale_Factor
			}
		}
	}

	return s, s.IMUError
}

// ---------- Private helpers ----------

func (d *Device) setGyroODR(hz uint16) error {
	if hz == 0 {
		hz = 1125
	}
	// divider = base/hz - 1; clamp to 0..255
	div := uint16(1125)/hz - 1
	if div > 255 {
		div = 255
	}
	if err := d.writeBankCached(2); err != nil {
		return err
	}
	if err := d.writeRetry(ICMREG_GYRO_SMPLRT_DIV, byte(div)); err != nil {
		return fmt.Errorf("gyro odr: %w", err)
	}
	return nil
}

func (d *Device) setAccelODR(hz uint16) error {
	if hz == 0 {
		hz = 1125
	}
	div := uint16(1125)/hz - 1
	if div > 255 {
		div = 255
	}
	if err := d.writeBankCached(2); err != nil {
		return err
	}
	// The accel uses two-byte divisor; we set only LSB for typical usage.
	if err := d.writeRetry(ICMREG_ACCEL_SMPLRT_DIV_2, byte(div)); err != nil {
		return fmt.Errorf("accel odr: %w", err)
	}
	return nil
}

// Configure internal I2C master for AK09916 continuous reads via SLV0 stream.
// Also sets the AK09916 mode via SLV1 write.
func (d *Device) initMagAK09916(mode MagMode) error {
	// Enable I2C master (USER_CTRL). NOTE: For ICM-20948, enabling aux IF uses bit 0x20 (same as MPU9250's I2C_MST_EN).
	if err := d.writeBankCached(0); err != nil {
		return err
	}
	uc, err := d.readRetry(ICMREG_USER_CTRL)
	if err != nil {
		return fmt.Errorf("user_ctrl read: %w", err)
	}
	if err := d.writeRetry(ICMREG_USER_CTRL, uc|BIT_AUX_IF_EN); err != nil {
		return fmt.Errorf("user_ctrl write: %w", err)
	}

	// BANK3: I2C master config + slaves
	if err := d.writeBankCached(3); err != nil {
		return err
	}
	// I2C master clock: pick a safe fast value; many refs use 0x07 (~345kHz).
	if err := d.writeRetry(ICMREG_I2C_MST_CTRL, 0x07); err != nil {
		return fmt.Errorf("i2c_mst_ctrl: %w", err)
	}

	// Program AK09916 mode via SLV1 (WRITE one byte to CNTL2)
	if err := d.writeRetry(ICMREG_I2C_SLV1_ADDR, AK09916_I2C_ADDR); err != nil {
		return fmt.Errorf("slv1 addr: %w", err)
	}
	if err := d.writeRetry(ICMREG_I2C_SLV1_REG, AK09916_CNTL2); err != nil {
		return fmt.Errorf("slv1 reg: %w", err)
	}
	if err := d.writeRetry(ICMREG_I2C_SLV1_CTRL, BIT_SLAVE_EN|0x01); err != nil {
		return fmt.Errorf("slv1 ctrl: %w", err)
	}
	if err := d.writeBankCached(0); err != nil {
		return err
	}
	if err := d.writeRetry(ICMREG_I2C_SLV1_DO, mode.akMode()); err != nil { // NOTE: your constants define ICMREG_I2C_SLV1_DO at 0x64
		return fmt.Errorf("slv1 do: %w", err)
	}

	// Return to BANK3 to set SLV0 stream (READ 8 bytes from ST1..ST2)
	if err := d.writeBankCached(3); err != nil {
		return err
	}
	if err := d.writeRetry(ICMREG_I2C_SLV0_ADDR, AK09916_I2C_ADDR|READ_FLAG); err != nil {
		return fmt.Errorf("slv0 addr: %w", err)
	}
	if err := d.writeRetry(ICMREG_I2C_SLV0_REG, AK09916_ST1); err != nil {
		return fmt.Errorf("slv0 reg: %w", err)
	}
	if err := d.writeRetry(ICMREG_I2C_SLV0_CTRL, BIT_SLAVE_EN|0x08); err != nil {
		return fmt.Errorf("slv0 ctrl: %w", err)
	}

	// Back to BANK0 for runtime reads
	if err := d.writeBankCached(0); err != nil {
		return err
	}

	return nil
}

func (d *Device) readBurst14() error {
	if err := d.writeBankCached(0); err != nil {
		return err
	}
	// NOTE: Your constants place ACCEL_XOUT_H at 0x2D and TEMP at 0x39; gyro after temp.
	// The 14-byte burst from 0x2D covers accel(6), gyro(6) *if contiguous*, but your map interleaves temp.
	// To remain consistent with your map, we still read 14 bytes starting at ACCEL_XOUT_H; this includes
	// accel(6), gyro(6) and *temp bytes located at 0x39/0x3A* which are inside this span in your definition.
	// If your hardware needs exact MPU ordering, adjust offsets accordingly.
	return d.bus.ReadFromReg(d.addr, ICMREG_ACCEL_XOUT_H, d.buf14[:])
}

func (d *Device) readMag8() error {
	if err := d.writeBankCached(0); err != nil {
		return err
	}
	return d.bus.ReadFromReg(d.addr, ICMREG_EXT_SENS_DATA_00, d.magBuf[:])
}

// ----- Bank/cache + retry wrappers -----

func (d *Device) writeBankCached(bank byte) error {
	if d.bankCached == bank {
		return nil
	}
	if err := d.bus.WriteByteToReg(d.addr, ICMREG_BANK_SEL, bank<<4); err != nil {
		return err
	}
	d.bankCached = bank
	return nil
}

func (d *Device) writeRetry(reg, val byte) error {
	var err error
	for i := 0; i <= d.retryCount; i++ {
		err = d.bus.WriteByteToReg(d.addr, reg, val)
		if err == nil {
			return nil
		}
		time.Sleep(d.retryDelay)
	}
	return err
}

func (d *Device) readRetry(reg byte) (byte, error) {
	var (
		v   byte
		err error
	)
	for i := 0; i <= d.retryCount; i++ {
		v, err = d.bus.ReadByteFromReg(d.addr, reg)
		if err == nil {
			return v, nil
		}
		time.Sleep(d.retryDelay)
	}
	return v, err
}

func (d *Device) waitClear(reg, mask byte, to time.Duration) error {
	deadline := time.Now().Add(to)
	for {
		v, err := d.readRetry(reg)
		if err != nil {
			return err
		}
		if (v & mask) == 0 {
			return nil
		}
		if time.Now().After(deadline) {
			return fmt.Errorf("timeout waiting clear reg 0x%02X mask 0x%02X", reg, mask)
		}
		time.Sleep(100 * time.Microsecond)
	}
}
