package ahrs

import (
	"log"
	"math"
)

const (
	minDT                      = 1e-6 // Below this time interval, don't recalculate
	maxDT                      = 10.0 // Above this time interval, re-initialize--too stale
	Pi      = math.Pi
	G       = 32.1740      //ft/s2
	Small   = 1e-9
	Big     = 1e9
	R2D     = 180 / Pi
	D2R		= Pi / 180
	Invalid float64  = 9999999 // large
	gyroCalDuration = 200  // Number of samples to collect (2 seconds at 100Hz)
)

type State struct {
		T float64 // Time when state last updated
}

// SimpleState now implements the Madgwick AHRS algorithm using quaternions
// Renamed for drop-in compatibility but uses Madgwick internally
type SimpleState struct {
	State
	T float64 // Time when state last updated
	tW                            float64                // Time of last GPS reading

	// Quaternion representing orientation (w, x, y, z) - Madgwick implementation
	q0, q1, q2, q3 float64
	gLoad, slipSkid, turnRate, heading float64

	// Madgwick filter parameters
	beta           float64  // Algorithm gain (proportional to gyro measurement error)
	
	
	// System state
	initialized bool
	valid       bool
	lastUpdate  float64

	needsInitialization           bool                   // Rather than computing, initialize
	logMap                        map[string]interface{} // Map only for analysis/debugging
	
	// Add gyro bias fields
	gyroBias                      [3]float64             // Gyro bias in deg/s (B1, B2, B3)
	needsGyroCal                  bool                   // Flag to trigger gyro calibration
	gyroCalSamples               int                    // Number of calibration samples collected
	gyroCalSum                   [3]float64             // Sum of gyro readings during calibration
	
	// Add magnetometer calibration
    magOffset                     [3]float64  // Hard iron offsets
    magScale                      [3]float64  // Soft iron scale factors
    magCalibrated                 bool        // Whether mag calibration is applied
}

type Measurement struct { // Order here also defines order in the matrices below
	UValid, WValid, SValid, MValid bool // Do we have valid airspeed, GPS, accel/gyro, and magnetometer readings?
	A1, A2, A3 float64 // Vector holding accelerometer readings, G, aircraft (accelerated) frame
	B1, B2, B3 float64 // Vector of gyro rates in roll, pitch, heading axes, °/s, aircraft (accelerated) frame
	M1, M2, M3 float64 // Vector of magnetometer readings, µT, aircraft (accelerated) frame y, x, and z axes
	TW, TU, T  float64 // Timestamp of GPS, airspeed and sensor readings
}

/////////////////////////////////////////////////////////////////////////
// Constructor functions
/////////////////////////////////////////////////////////////////////////

//NewSimpleAHRS returns a new Simple AHRS object (now using Madgwick algorithm).
// It is initialized with a beginning sensor orientation quaternion f0.
func NewSimpleAHRS() (s *SimpleState) {
	s = new(SimpleState)

	s.needsInitialization = true
	s.needsGyroCal = true

	// Initialize quaternion to identity (no rotation)
	s.q0, s.q1, s.q2, s.q3 = 1.0, 0.0, 0.0, 0.0
	
	// Initialize Madgwick filter parameters
	s.beta = 0.1   // Algorithm gain (tune based on gyro noise characteristics)
	
	// Initialize magnetometer calibration with your values M2, M1, -M3 (mx, my, mz)
    s.magOffset = [3]float64{5154, 589, 1209}
    s.magScale = [3]float64{0.000158780565259, 0.000160901045857, 0.000143802128271}
    s.magCalibrated = true

	s.logMap = make(map[string]interface{})
	updateLogMap(s, NewMeasurement(), s.logMap)
	return
}

//NewAHRS returns a new Simple AHRS object (now using Madgwick algorithm).
func NewAHRS() (s *SimpleState) {
	return NewSimpleAHRS()
}

// NewMeasurement returns a pointer to an empty AHRS Measurement.
// Uncertainty matrix and variance accumulators are properly initialized.
func NewMeasurement() *Measurement {
	m := new(Measurement)
	return m
}

/////////////////////////////////////////////////////////////////////////
// Initialization
/////////////////////////////////////////////////////////////////////////

// init initializes the filter with the first measurement
func (s *SimpleState) init(m *Measurement) {
	log.Printf("Initializing Madgwick Filter")

	s.T = m.T
	s.tW = m.TW

	// Extract accelerometer readings
	ax, ay, az := m.A1, m.A2, m.A3
	
	// Extract magnetometer readings
	mx, my, mz := m.M2, m.M1, -m.M3

	// Apply magnetometer calibration if enabled
    if s.magCalibrated {
        mx = (mx - s.magOffset[0]) * s.magScale[0]
        my = (my - s.magOffset[1]) * s.magScale[1]
        mz = (mz - s.magOffset[2]) * s.magScale[2]
    }
	
	// Check magnetometer validity
	mmag := math.Sqrt(mx*mx + my*my + mz*mz)
	magValid := mmag >= 0.9 && mmag <= 1.1

	// Initialize orientation from accelerometer and magnetometer
	if magValid {
		// Use MARG (9-DOF) initialization
		s.initializeMARG(ax, ay, az, mx, my, mz)
	} else {
		log.Printf("High magnetometer error")
		return
	}

	s.needsInitialization = false

	roll, pitch, heading :=  s.updateEulerAngles()
	
	log.Printf("AHRS: Initialized with q=[%.3f, %.3f, %.3f, %.3f] Roll: %.1f°, Pitch: %.1f°, Heading: %.1f°", s.q0, s.q1, s.q2, s.q3, roll*R2D, pitch*R2D, heading*R2D)
}


// initializeMARG initializes orientation using accelerometer and magnetometer (9-DOF)
func (s *SimpleState) initializeMARG(ax, ay, az, mx, my, mz float64) {
	// Normalize accelerometer
	normA := math.Sqrt(ax*ax + ay*ay + az*az)
	if normA > 0 {
		ax /= normA
		ay /= normA
		az /= normA
	}
	
	// Calculate initial roll and pitch from accelerometer
	roll := math.Atan2(ay, az)
	pitch := math.Atan2(ax, math.Sqrt(ay*ay + az*az))
	
	// Calculate initial yaw from magnetometer
	// Transform magnetometer to horizontal plane
	mxh := mx*math.Cos(pitch) + my*math.Sin(roll)*math.Sin(pitch) + mz*math.Cos(roll)*math.Sin(pitch)
	myh := my*math.Cos(roll) - mz*math.Sin(roll)
	
	yaw := math.Atan2(myh, mxh)
	
	// Convert to quaternion
	s.eulerToQuaternion(roll, pitch, yaw)
}

// eulerToQuaternion converts Euler angles to quaternion
func (s *SimpleState) eulerToQuaternion(roll, pitch, yaw float64) {
	// Convert to half angles
	cr := math.Cos(roll * 0.5)
	sr := math.Sin(roll * 0.5)
	cp := math.Cos(pitch * 0.5)
	sp := math.Sin(pitch * 0.5)
	cy := math.Cos(yaw * 0.5)
	sy := math.Sin(yaw * 0.5)
	
	// Calculate quaternion components
	s.q0 = cr*cp*cy + sr*sp*sy
	s.q1 = sr*cp*cy - cr*sp*sy
	s.q2 = cr*sp*cy + sr*cp*sy
	s.q3 = cr*cp*sy - sr*sp*cy
}















// Gyro calibration - collect stationary samples to estimate bias
func (s *SimpleState) calibrateGyro(m *Measurement) bool {
	if !m.SValid {
		return false  // Can't calibrate without gyro data
	}
	
	// Accumulate samples
	s.gyroCalSum[0] += m.B1
	s.gyroCalSum[1] += m.B2
	s.gyroCalSum[2] += m.B3
	s.gyroCalSamples++
	
	if s.gyroCalSamples >= gyroCalDuration {
		// Calculate average bias
		s.gyroBias[0] = s.gyroCalSum[0] / float64(s.gyroCalSamples)
		s.gyroBias[1] = s.gyroCalSum[1] / float64(s.gyroCalSamples)
		s.gyroBias[2] = s.gyroCalSum[2] / float64(s.gyroCalSamples)
		
		// Reset calibration state
		s.needsGyroCal = false
		s.gyroCalSamples = 0
		s.gyroCalSum = [3]float64{0, 0, 0}
		
		log.Printf("AHRS: Gyro calibration complete")
		log.Printf("AHRS: Gyro biases: X=%.3f Y=%.3f Z=%.3f deg/s", 
			s.gyroBias[0], s.gyroBias[1], s.gyroBias[2])
		
		return true
	}
	
	return false  // Still calibrating
}

// ResetGyroCal triggers a new gyro calibration
func (s *SimpleState) ResetGyroCal() {
	s.needsGyroCal = true
	s.gyroCalSamples = 0
	s.gyroCalSum = [3]float64{0, 0, 0}
	s.gyroBias = [3]float64{0, 0, 0}
	log.Printf("AHRS: Gyro calibration reset - keep IMU stationary!")
}











































































/////////////////////////////////////////////////////////////////////////
// Run
/////////////////////////////////////////////////////////////////////////

// Compute performs both prediction and update steps
func (s *SimpleState) Compute(m *Measurement) {
	// First check if we need to calibrate gyro
	if s.needsGyroCal {
		gyroCalComplete := s.calibrateGyro(m)
		if !gyroCalComplete {
			log.Printf("Gyro Calibrating...")
			return  // Don't run AHRS until gyro calibration is done
		}
	}

	if s.needsInitialization {
		log.Printf("AHRS Initializing...")
		s.init(m)
		return
	}

	s.Update(m)

	// Update log map for debugging
	updateLogMap(s, m, s.logMap)
}






// Madgwick AHRS algorithm implementation
func (s *SimpleState) Update(m *Measurement) {
	dt := m.T - s.T
	dtw := m.TW - s.tW
	log.Printf("dt %f dtw %f", dt, dtw)

	if dt > maxDT || dtw > maxDT {
		log.Printf("ERROR - timestep too large for AHRS")
		s.init(m)
		return
	}
	
	if dt < minDT {
		return  // Skip if time step too small
	}

	// Extract and convert sensor data
	gx := (m.B1 - s.gyroBias[0]) * D2R  // Convert to rad/s and remove bias
	gy := (m.B2 - s.gyroBias[1]) * D2R
	gz := (m.B3 - s.gyroBias[2]) * D2R
	
	ax := m.A1
	ay := m.A2  
	az := m.A3
	
	mx := m.M2
	my := m.M1
	mz := -m.M3

	// Apply magnetometer calibration
	if s.magCalibrated {
		mx = (mx - s.magOffset[0]) * s.magScale[0]
		my = (my - s.magOffset[1]) * s.magScale[1]
		mz = (mz - s.magOffset[2]) * s.magScale[2]
	}

	// Check if we have valid magnetometer data
	mmag := math.Sqrt(mx*mx + my*my + mz*mz)
	magValid := m.MValid && mmag >= 0.9 && mmag <= 1.1

	// Run the appropriate Madgwick algorithm
	if magValid {
		s.madgwickAHRS(gx, gy, gz, ax, ay, az, mx, my, mz, dt)
	} else {
		return
	}

	// Update timing
	s.T = m.T
	s.tW = m.TW

	// Update derived quantities
	s.updateDerivedQuantities(m)
}

// madgwickAHRS implements the full 9-DOF Madgwick algorithm with magnetometer
func (s *SimpleState) madgwickAHRS(gx, gy, gz, ax, ay, az, mx, my, mz, dt float64) {
	// Local system variables
	rNorm := 0.0
	s_x, s_y, s_z := 0.0, 0.0, 0.0  // Sensor frame direction cosines
	qDot1, qDot2, qDot3, qDot4 := 0.0, 0.0, 0.0, 0.0  // Quaternion derivative
	hx, hy, bx, bz := 0.0, 0.0, 0.0, 0.0
	
	// Use IMU algorithm if magnetometer measurement invalid
	if !((mx == 0.0) && (my == 0.0) && (mz == 0.0)) {
		// Normalise accelerometer measurement
		aNorm := math.Sqrt(ax*ax + ay*ay + az*az)
		ax /= aNorm
		ay /= aNorm
		az /= aNorm

		// Normalise magnetometer measurement
		mNorm := math.Sqrt(mx*mx + my*my + mz*mz)
		mx /= mNorm
		my /= mNorm
		mz /= mNorm

		// Auxiliary variables to avoid repeated arithmetic
		_2q0mx := 2.0 * s.q0 * mx
		_2q0my := 2.0 * s.q0 * my
		_2q0mz := 2.0 * s.q0 * mz
		_2q1mx := 2.0 * s.q1 * mx
		_2q0 := 2.0 * s.q0
		_2q1 := 2.0 * s.q1
		_2q2 := 2.0 * s.q2
		_2q3 := 2.0 * s.q3
		_2q0q2 := 2.0 * s.q0 * s.q2
		_2q2q3 := 2.0 * s.q2 * s.q3
		q0q0 := s.q0 * s.q0
		q0q1 := s.q0 * s.q1
		q0q2 := s.q0 * s.q2
		q0q3 := s.q0 * s.q3
		q1q1 := s.q1 * s.q1
		q1q2 := s.q1 * s.q2
		q1q3 := s.q1 * s.q3
		q2q2 := s.q2 * s.q2
		q2q3 := s.q2 * s.q3
		q3q3 := s.q3 * s.q3

		// Reference direction of Earth's magnetic field
		hx = mx * q0q0 - _2q0my * s.q3 + _2q0mz * s.q2 + mx * q1q1 + _2q1 * my * s.q2 + _2q1 * mz * s.q3 - mx * q2q2 - mx * q3q3
		hy = _2q0mx * s.q3 + my * q0q0 - _2q0mz * s.q1 + _2q1mx * s.q2 - my * q1q1 + my * q2q2 + _2q2 * mz * s.q3 - my * q3q3
		_2bx := math.Sqrt(hx * hx + hy * hy)
		bx = _2bx * 0.5
		bz = -_2q0mx * s.q2 + _2q0my * s.q1 + mz * q0q0 + _2q1mx * s.q3 - mz * q1q1 + _2q2 * my * s.q3 - mz * q2q2 + mz * q3q3
		_4bx := 2.0 * bx
		_4bz := 2.0 * bz
		_8bx := 4.0 * bx
		_8bz := 4.0 * bz

		// Gradient decent algorithm corrective step
		s_x = -_2q2 * (2.0 * q1q3 - _2q0q2 - ax) + _2q1 * (2.0 * q0q1 + _2q2q3 - ay) - _4bz * s.q2 * (_4bx * (0.5 - q2q2 - q3q3) + _4bz * (q1q3 - q0q2) - mx) + (-_4bx * s.q3 + _4bz * s.q1) * (_4bx * (q1q2 - q0q3) + _4bz * (q0q1 + q2q3) - my) + _4bx * s.q2 * (_4bx * (q0q2 + q1q3) + _4bz * (0.5 - q1q1 - q2q2) - mz)
		s_y = _2q3 * (2.0 * q1q3 - _2q0q2 - ax) + _2q0 * (2.0 * q0q1 + _2q2q3 - ay) - 4.0 * s.q1 * (1.0 - 2.0 * q1q1 - 2.0 * q2q2 - az) + _4bz * s.q3 * (_4bx * (0.5 - q2q2 - q3q3) + _4bz * (q1q3 - q0q2) - mx) + (_4bx * s.q2 + _4bz * s.q0) * (_4bx * (q1q2 - q0q3) + _4bz * (q0q1 + q2q3) - my) + (_4bx * s.q3 - _8bz * s.q1) * (_4bx * (q0q2 + q1q3) + _4bz * (0.5 - q1q1 - q2q2) - mz)
		s_z = -_2q0 * (2.0 * q1q3 - _2q0q2 - ax) + _2q3 * (2.0 * q0q1 + _2q2q3 - ay) - 4.0 * s.q2 * (1.0 - 2.0 * q1q1 - 2.0 * q2q2 - az) + (-_8bx * s.q2 - _4bz * s.q0) * (_4bx * (0.5 - q2q2 - q3q3) + _4bz * (q1q3 - q0q2) - mx) + (_4bx * s.q1 + _4bz * s.q3) * (_4bx * (q1q2 - q0q3) + _4bz * (q0q1 + q2q3) - my) + (_4bx * s.q0 - _8bz * s.q2) * (_4bx * (q0q2 + q1q3) + _4bz * (0.5 - q1q1 - q2q2) - mz)
		s_w := -_2q1 * (2.0 * q1q3 - _2q0q2 - ax) - _2q2 * (2.0 * q0q1 + _2q2q3 - ay) + _4bz * s.q1 * (_4bx * (0.5 - q2q2 - q3q3) + _4bz * (q1q3 - q0q2) - mx) + (-_4bx * s.q0 + _4bz * s.q2) * (_4bx * (q1q2 - q0q3) + _4bz * (q0q1 + q2q3) - my) + _4bx * s.q1 * (_4bx * (q0q2 + q1q3) + _4bz * (0.5 - q1q1 - q2q2) - mz)
		rNorm = math.Sqrt(s_w*s_w + s_x*s_x + s_y*s_y + s_z*s_z) // normalise step magnitude
		s_w /= rNorm
		s_x /= rNorm
		s_y /= rNorm
		s_z /= rNorm

		// Apply feedback step
		qDot1 = 0.5*(-s.q1*gx - s.q2*gy - s.q3*gz) - s.beta*s_w
		qDot2 = 0.5*(s.q0*gx + s.q2*gz - s.q3*gy) - s.beta*s_x
		qDot3 = 0.5*(s.q0*gy - s.q1*gz + s.q3*gx) - s.beta*s_y
		qDot4 = 0.5*(s.q0*gz + s.q1*gy - s.q2*gx) - s.beta*s_z
	} else {
		return
	}

	// Integrate rate of change of quaternion to yield quaternion
	s.q0 += qDot1 * dt
	s.q1 += qDot2 * dt
	s.q2 += qDot3 * dt
	s.q3 += qDot4 * dt

	// Normalise quaternion
	rNorm = math.Sqrt(s.q0*s.q0 + s.q1*s.q1 + s.q2*s.q2 + s.q3*s.q3)
	s.q0 /= rNorm
	s.q1 /= rNorm
	s.q2 /= rNorm
	s.q3 /= rNorm
}





































































































































// updateEulerAngles converts quaternion to Euler angles
func (s *SimpleState) updateEulerAngles() (roll float64, pitch float64, heading float64) {
	// Roll (x-axis rotation)
	sinr_cosp := 2 * (s.q0*s.q1 + s.q2*s.q3)
	cosr_cosp := 1 - 2*(s.q1*s.q1 + s.q2*s.q2)
	roll = math.Atan2(sinr_cosp, cosr_cosp)

	// Pitch (y-axis rotation)
	sinp := 2 * (s.q0*s.q2 - s.q3*s.q1)
	pitch = 0
	if math.Abs(sinp) >= 1 {
		if sinp > 0 {
			pitch = math.Pi / 2 // 90 degrees
		} else {
			pitch = -math.Pi / 2 // -90 degrees
		}
	} else {
		pitch = math.Asin(sinp)
	}

	// Yaw (z-axis rotation)
	siny_cosp := 2 * (s.q0*s.q3 + s.q1*s.q2)
	cosy_cosp := 1 - 2*(s.q2*s.q2 + s.q3*s.q3)
	heading = math.Atan2(siny_cosp, cosy_cosp)
	
	// Ensure heading is in [0, 2π] range
	for heading < 0 {
		heading += 2 * Pi
	}
	for heading >= 2*Pi {
		heading -= 2 * Pi
	}
	return roll, pitch, heading
}









// updateDCM updates the Direction Cosine Matrix from quaternion
//func (s *SimpleState) updateDCM() {
	// Convert quaternion to DCM
//	q0q0 := s.q0 * s.q0
//	q0q1 := s.q0 * s.q1
//	q0q2 := s.q0 * s.q2
//	q0q3 := s.q0 * s.q3
//	q1q1 := s.q1 * s.q1
//	q1q2 := s.q1 * s.q2
//	q1q3 := s.q1 * s.q3
//	q2q2 := s.q2 * s.q2
//	q2q3 := s.q2 * s.q3
//	q3q3 := s.q3 * s.q3
//
//	s.dcm[0][0] = q0q0 + q1q1 - q2q2 - q3q3
//	s.dcm[0][1] = 2*(q1q2 - q0q3)
//	s.dcm[0][2] = 2*(q1q3 + q0q2)
//	s.dcm[1][0] = 2*(q1q2 + q0q3)
//	s.dcm[1][1] = q0q0 - q1q1 + q2q2 - q3q3
//	s.dcm[1][2] = 2*(q2q3 - q0q1)
//	s.dcm[2][0] = 2*(q1q3 - q0q2)
//	s.dcm[2][1] = 2*(q2q3 + q0q1)
//	s.dcm[2][2] = q0q0 - q1q1 - q2q2 + q3q3
//}

// updateDerivedQuantities calculates slip/skid, turn rate, g-load
func (s *SimpleState) updateDerivedQuantities(m *Measurement) {
	if !m.SValid {
		return
	}

	// G-load is the magnitude of acceleration
	_, ay, az := m.A1, m.A2, m.A3
	s.gLoad = 0.9*s.gLoad + 0.1*az

	// Slip/skid angle (simplified)
	s.slipSkid = 0.9*s.slipSkid + 0.1 * math.Atan2(-ay, az) * R2D

	// Turn rate from gyro (bias corrected)
	s.turnRate = 0.9*s.turnRate + 0.1*((m.B3*D2R) - s.gyroBias[2]*D2R) * R2D // Convert back to deg/s
}

// Valid returns whether the current state is a valid estimate
func (s *SimpleState) Valid() (ok bool) {
	// Check for NaN in quaternion
	if math.IsNaN(s.q0) || math.IsNaN(s.q1) || math.IsNaN(s.q2) || math.IsNaN(s.q3) {
		return false
	}
	
	// Check quaternion magnitude (should be close to 1)
	qMag := math.Sqrt(s.q0*s.q0 + s.q1*s.q1 + s.q2*s.q2 + s.q3*s.q3)
	if math.Abs(qMag - 1.0) > 0.1 {
		return false
	}
	
	return !s.needsInitialization
}

// Reset restarts the algorithm from scratch
func (s *SimpleState) Reset() {
	s.needsInitialization = true
	s.q0, s.q1, s.q2, s.q3 = 1.0, 0.0, 0.0, 0.0
}

// RollPitchHeading returns the current attitude values
func (s *SimpleState) RollPitchHeading() (roll float64, pitch float64, heading float64) {
	roll, pitch, heading = s.updateEulerAngles()
	return
}

// MagHeading returns the magnetic heading in degrees
func (s *SimpleState) MagHeading() (hdg float64) {
	_, _, hdg = s.updateEulerAngles()
	return hdg
}

// SlipSkid returns the slip/skid angle in degrees
func (s *SimpleState) SlipSkid() (slipSkid float64) {
	return s.slipSkid
}

// RateOfTurn returns the turn rate in degrees per second
func (s *SimpleState) RateOfTurn() (turnRate float64) {
	return s.turnRate
}

// GLoad returns the current G load, in G's
func (s *SimpleState) GLoad() (gLoad float64) {
	return s.gLoad
}

// GetState returns the state of the system
func (s *SimpleState) GetState() *State {
	return &s.State
}

// GetLogMap returns a map providing current state and measurement values for analysis
func (s *SimpleState) GetLogMap() (p map[string]interface{}) {
	return s.logMap
}

// GetQuaternion returns the current orientation quaternion
func (s *SimpleState) GetQuaternion() (q0, q1, q2, q3 float64) {
	return s.q0, s.q1, s.q2, s.q3
}

// updateLogMap updates the logging map for analysis
func updateLogMap(s *SimpleState, m *Measurement, p map[string]interface{}) {
	var simpleLogMap = map[string]func(s *SimpleState, m *Measurement) float64{
		"Ta":                func(s *SimpleState, m *Measurement) float64 { return s.T },
		"TWa":               func(s *SimpleState, m *Measurement) float64 { return s.tW },
//		"Roll":              func(s *SimpleState, m *Measurement) float64 { return s.roll * R2D },
//		"Pitch":             func(s *SimpleState, m *Measurement) float64 { return s.pitch * R2D },
//		"Heading":           func(s *SimpleState, m *Measurement) float64 { return s.heading * R2D },
		"Q0":                func(s *SimpleState, m *Measurement) float64 { return s.q0 },
		"Q1":                func(s *SimpleState, m *Measurement) float64 { return s.q1 },
		"Q2":                func(s *SimpleState, m *Measurement) float64 { return s.q2 },
		"Q3":                func(s *SimpleState, m *Measurement) float64 { return s.q3 },
		"T":                 func(s *SimpleState, m *Measurement) float64 { return m.T },
		"TW":                func(s *SimpleState, m *Measurement) float64 { return m.TW },
		"A1":                func(s *SimpleState, m *Measurement) float64 { return m.A1 },
		"A2":                func(s *SimpleState, m *Measurement) float64 { return m.A2 },
		"A3":                func(s *SimpleState, m *Measurement) float64 { return m.A3 },
		"B1":                func(s *SimpleState, m *Measurement) float64 { return m.B1 },
		"B2":                func(s *SimpleState, m *Measurement) float64 { return m.B2 },
		"B3":                func(s *SimpleState, m *Measurement) float64 { return m.B3 },
		"M1":                func(s *SimpleState, m *Measurement) float64 { return m.M1 },
		"M2":                func(s *SimpleState, m *Measurement) float64 { return m.M2 },
		"M3":                func(s *SimpleState, m *Measurement) float64 { return m.M3 },
		"GyroBiasX":         func(s *SimpleState, m *Measurement) float64 { return s.gyroBias[0] },
		"GyroBiasY":         func(s *SimpleState, m *Measurement) float64 { return s.gyroBias[1] },
		"GyroBiasZ":         func(s *SimpleState, m *Measurement) float64 { return s.gyroBias[2] },
		"TurnRate":          func(s *SimpleState, m *Measurement) float64 { return s.turnRate },
		"GLoad":             func(s *SimpleState, m *Measurement) float64 { return s.gLoad },
		"SlipSkid":          func(s *SimpleState, m *Measurement) float64 { return s.slipSkid },
	}

	for k := range simpleLogMap {
		p[k] = simpleLogMap[k](s, m)
	}
}

var SimpleJSONConfig = `{
  "State": [
    ["Roll", "RollGPS", "RollGyr", "RollActual", 0],
    ["Pitch", "PitchGPS", "PitchGyr", "PitchActual", 0],
    ["Heading", "HeadingGPS", "HeadingGyr", "HeadingActual", null],
    ["turnRate", null, null, "turnRateActual", 0],
    ["gLoad", null, null, "gLoadActual", 1],
    ["slipSkid", null, null, "slipSkidActual", 0],
    ["GroundSpeed", null, null, null, 0],
    ["T", null, null, null, null],
    ["Q0", "Q0GPS", "Q0Gyr", "Q0Actual", null],
    ["Q1", "Q1GPS", "Q1Gyr", "Q1Actual", null],
    ["Q2", "Q2GPS", "Q2Gyr", "Q2Actual", null],
    ["Q3", "Q3GPS", "Q3Gyr", "Q3Actual", null],
    ["Z1", null, null, "Z1Actual", 0],
    ["Z2", null, null, "Z2Actual", 0],
    ["Z3", null, null, "Z3Actual", -1],
    ["C1", null, null, "C1Actual", 0],
    ["C2", null, null, "C2Actual", 0],
    ["C3", null, null, "C3Actual", 0],
    ["H1", null, null, "H1Actual", 0],
    ["H2", null, null, "H2Actual", 0],
    ["H3", null, null, "H3Actual", 0],
    ["D1", null, null, "D1Actual", 0],
    ["D2", null, null, "D2Actual", 0],
    ["D3", null, null, "D3Actual", 0]
  ],
  "Measurement": [
    ["W1", "W1a", 0],
    ["W2", "W2a", 0],
    ["W3", "W3a", 0],
    ["A1", null, 0],
    ["A2", null, 0],
    ["A3", null, 0],
    ["B1", null, 0],
    ["B2", null, 0],
    ["B3", null, 0],
    ["M1", null, 0],
    ["M2", null, 0],
    ["M3", null, 0]
  ]
}`

// AHRSProvider defines an AHRS (Kalman or other) algorithm, such as ahrs_kalman, ahrs_simple, etc.
type AHRSProvider interface {
	// Compute runs both the "predict" and "update" stages of the algorithm, for convenience.
	Compute(m *Measurement)
	// SetSensorQuaternion changes the AHRS algorithm's sensor quaternion F.
	Valid() bool
	// Reset restarts the algorithm from scratch.
	Reset()
	// RollPitchHeading returns the current attitude values as estimated by the Kalman algorithm.
	RollPitchHeading() (roll float64, pitch float64, heading float64)
	// MagHeading returns the current magnetic heading in degrees as estimated by the Kalman algorithm.
	MagHeading() (hdg float64)
	// SlipSkid returns the slip/skid angle in degrees as estimated by the Kalman algorithm.
	SlipSkid() (slipSkid float64)
	// RateOfTurn returns the turn rate in degrees per second as estimated by the Kalman algorithm.
	RateOfTurn() (turnRate float64)
	// GLoad returns the current G load, in G's as estimated by the Kalman algorithm.
	GLoad() (gLoad float64)
	// GetState returns all the information about the current state.
	GetState() *State
	// GetLogMap returns a map customized for each AHRSProvider algorithm to provide more detailed information
	// for debugging and logging.
	GetLogMap() map[string]interface{}
}