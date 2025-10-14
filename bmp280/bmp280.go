// Package bmp280: minimal BMP280 driver (sync, no goroutines/chans).
package bmp280

import (
	"errors"
	"time"

	"github.com/kidoman/embd"
)

const (
	Addr0x76 = 0x76
	Addr0x77 = 0x77

	regCalib  = 0x88 // 24 bytes (T1..T3, P1..P9)
	regID     = 0xD0 // should be 0x58 for BMP280
	regReset  = 0xE0
	regStatus = 0xF3
	regCtrl   = 0xF4
	regConfig = 0xF5

	regPressMSB = 0xF7 // press[19:12]
	regPressLSB = 0xF8 // press[11:4]
	regPressXLS = 0xF9 // press[3:0] (upper nibble)

	regTempMSB = 0xFA
	regTempLSB = 0xFB
	regTempXLS = 0xFC

	softReset = 0xB6

	// ctrl_meas bits
	osrsT1 = 0x20 // temp oversample x1 (5:7)
	osrsP1 = 0x04 // press oversample x1 (2:4)
	modeNormal = 0x03

	// config bits
	standby_125ms = 0x02 << 5 // t_sb
	filter_off    = 0x00 << 2 // IIR off
)

type Reading struct {
	Time       time.Time
	TempC      float64
	PressurePa float64
}

// datasheet-defined calibration; fixed layout, don’t use maps.
type calib struct {
	T1 uint16
	T2 int16
	T3 int16
	P1 uint16
	P2 int16
	P3 int16
	P4 int16
	P5 int16
	P6 int16
	P7 int16
	P8 int16
	P9 int16
}

type Device struct {
	bus   embd.I2CBus // interface, not *interface
	addr  byte
	c     calib
	tFine int32
}

// New resets, loads calibration, and puts the device in a sane config.
func New(bus embd.I2CBus, addr byte) (*Device, error) {
	d := &Device{bus: bus, addr: addr}

	// Soft reset
	if err := d.write(regReset, softReset); err != nil {
		return nil, err
	}
	time.Sleep(2 * time.Millisecond)

	// Read chip ID (BMP280 is 0x58). Some clones report others; don’t hard fail,
	// but you *can* check and warn at a higher level if you want.
	id := make([]byte, 1)
	if err := d.read(regID, id); err != nil {
		return nil, err
	}

	// Load calibration
	if err := d.readCalibration(); err != nil {
		return nil, err
	}

	// Configure: osrs_t=x1, osrs_p=x1, normal mode; IIR off, 125 ms standby
	if err := d.write(regCtrl, osrsT1|osrsP1|modeNormal); err != nil {
		return nil, err
	}
	if err := d.write(regConfig, standby_125ms|filter_off); err != nil {
		return nil, err
	}
	return d, nil
}

func (d *Device) Read() (Reading, error) {
	// Read raw pressure+temp (6 bytes starting at 0xF7).
	buf := make([]byte, 6)
	if err := d.read(regPressMSB, buf); err != nil {
		return Reading{}, err
	}

	// 20-bit unsigned values
	rawPress := int32(buf[0])<<12 | int32(buf[1])<<4 | int32(buf[2])>>4
	rawTemp  := int32(buf[3])<<12 | int32(buf[4])<<4 | int32(buf[5])>>4

	tempC := d.compTemp(rawTemp)
	pressPa := d.compPress(int64(rawPress))

	return Reading{
		Time:       time.Now(),
		TempC:      tempC,
		PressurePa: pressPa,
	}, nil
}

func (d *Device) Close() error {
	// Optional: set sleep mode
	// _ = d.write(regCtrl, 0)
	return nil
}

// --- internals ---

func (d *Device) write(reg, val byte) error {
	return d.bus.WriteByteToReg(d.addr, reg, val)
}

func (d *Device) read(reg byte, dst []byte) error {
	if len(dst) == 0 {
		return errors.New("zero-length read")
	}
	return d.bus.ReadFromReg(d.addr, reg, dst)
}

func (d *Device) readCalibration() error {
	raw := make([]byte, 24)
	if err := d.read(regCalib, raw); err != nil {
		return err
	}
	u16 := func(lo, hi byte) uint16 { return uint16(hi)<<8 | uint16(lo) }
	s16 := func(lo, hi byte) int16 { return int16(u16(lo, hi)) }

	d.c.T1 = u16(raw[0], raw[1])
	d.c.T2 = s16(raw[2], raw[3])
	d.c.T3 = s16(raw[4], raw[5])

	d.c.P1 = u16(raw[6], raw[7])
	d.c.P2 = s16(raw[8], raw[9])
	d.c.P3 = s16(raw[10], raw[11])
	d.c.P4 = s16(raw[12], raw[13])
	d.c.P5 = s16(raw[14], raw[15])
	d.c.P6 = s16(raw[16], raw[17])
	d.c.P7 = s16(raw[18], raw[19])
	d.c.P8 = s16(raw[20], raw[21])
	d.c.P9 = s16(raw[22], raw[23])
	return nil
}

// Bosch compensation from datasheet (integer math -> float64 at the end).

func (d *Device) compTemp(adcT int32) float64 {
	var1 := (((adcT>>3) - int32(d.c.T1<<1)) * int32(d.c.T2)) >> 11
	var2 := (((((adcT>>4) - int32(d.c.T1)) * ((adcT>>4) - int32(d.c.T1))) >> 12) * int32(d.c.T3)) >> 14
	d.tFine = var1 + var2
	t := (d.tFine*5 + 128) >> 8 // °C * 100
	return float64(t) / 100.0
}

func (d *Device) compPress(adcP int64) float64 {
	var1 := int64(d.tFine) - 128000
	var2 := var1 * var1 * int64(d.c.P6)
	var2 += (var1 * int64(d.c.P5)) << 17
	var2 += int64(d.c.P4) << 35
	var1 = ((var1*var1*int64(d.c.P3)) >> 8) + ((var1 * int64(d.c.P2)) << 12)
	var1 = (((int64(1) << 47) + var1) * int64(d.c.P1)) >> 33
	if var1 == 0 {
		return 0 // avoid div by zero
	}
	p := int64(1048576) - adcP
	p = (((p << 31) - var2) * 3125) / var1
	var1 = (int64(d.c.P9) * (p >> 13) * (p >> 13)) >> 25
	var2 = (int64(d.c.P8) * p) >> 19
	p = ((p + var1 + var2) >> 8) + (int64(d.c.P7) << 4)

	// Datasheet yields Pa*256; convert to Pa.
	return float64(p) / 256.0
}
