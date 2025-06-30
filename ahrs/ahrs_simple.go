package ahrs

import (
	"log"
	"math"
//	"fmt"
//	"github.com/skelterjohn/go.matrix"
)

const (
	minDT                      = 1e-6 // Below this time interval, don't recalculate
	maxDT                      = 10.0 // Above this time interval, re-initialize--too stale
	Pi      = math.Pi
	G       = 32.1740
	Small   = 1e-9
	Big     = 1e9
	R2D     = 180 / Pi
	D2R		= Pi / 180
	Invalid float64  = 9999999 // 2**15-1
	gyroCalDuration = 200  // Number of samples to collect (2 seconds at 100Hz)
)


type State struct {
	T float64 // Time when state last updated
	F0, F1, F2, F3 float64 // quaternion rotating aircraft frame to sensor frame
	f11, f12, f13 float64 // cached sensor-aircraft rotation matrix
	f21, f22, f23 float64
	f31, f32, f33 float64
}

type SimpleState struct {
	State
	tW                            float64                // Time of last GPS reading
	roll, pitch, heading          float64                // Fused attitude, Rad
	slipSkid                      float64                // Slip/Skid Angle, Rad
	gLoad                         float64                // G Load, G vertical
	turnRate                      float64                // turn rate, Rad/s
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
	//TODO westphae: track separate measurement timestamps for Gyro/Accel, Magnetometer, GPS, Baro
}

//NewSimpleAHRS returns a new Simple AHRS object.
// It is initialized with a beginning sensor orientation quaternion f0.
func NewAHRS() (s *SimpleState) {
	s = new(SimpleState)
	s.logMap = make(map[string]interface{})
	updateLogMap(s, NewMeasurement(), s.logMap)
	s.needsInitialization = true
	s.needsGyroCal = true

	// Initialize magnetometer calibration with your values M2, M1, M3 (mx, my, mz)
    s.magOffset = [3]float64{3140.8, 1824.9, -1533.5}
    s.magScale = [3]float64{.974484, 1.000000, 0.605036}
    s.magCalibrated = true

	return
}

// NewMeasurement returns a pointer to an empty AHRS Measurement.
// Uncertainty matrix and variance accumulators are properly initialized.
func NewMeasurement() *Measurement {
	m := new(Measurement)
	return m
}















func (s *SimpleState) init(m *Measurement) {
	log.Printf("Initializing")
	s.needsInitialization = false

	s.T = m.T
	s.tW = m.TW

	ax:=m.A1
	ay:=m.A2
	az:=m.A3
	mx:=m.M2
	my:=m.M1
	mz:=m.M3

	s.roll = math.Atan2(ay, az)
	s.pitch = math.Atan2(-ax, math.Sqrt(ay*ay + az*az))

    m1 := mx * math.Cos(s.pitch) + my * math.Sin(s.roll);           
    m2 := mx * math.Sin(s.roll) * math.Sin(s.pitch) + my * math.Cos(s.roll) - mz * math.Sin(s.roll) * math.Cos(s.pitch);
    s.heading = math.Atan2(m2, m1);
	for s.heading < 0 {
		s.heading += 2 * Pi
	}
	for s.heading >= 2*Pi {
		s.heading -= 2 * Pi
	}

	log.Printf("INIT: roll %f pitch %f yaw %f", s.roll*R2D, s.pitch*R2D, s.heading*R2D)
	
	s.F0, s.F1, s.F2, s.F3 = toQuaternion(s.roll, s.pitch, s.heading)
	s.calcRotationMatrices()

	//these were fixed to use body-fixed accelerations instead of inertial accelerations
	// Initialize Slip/Skid, Rate of Turn, and GLoad.
	s.slipSkid = math.Atan2(m.A2, m.A3)*R2D
	s.turnRate = 0
	s.gLoad = m.A3 
	log.Printf("INIT: Slipskid %f turnrate %f gload %f", s.slipSkid, s.turnRate, s.gLoad)
	updateLogMap(s, m, s.logMap)

}


// calibrateGyro collects stationary gyro readings to estimate bias
func (s *SimpleState) calibrateGyro(m *Measurement) bool {
	if !s.needsGyroCal {
		return true  // Calibration already complete
	}
	
	// Check if gyro data is valid
	if !m.SValid {
		log.Printf("AHRS: Gyro calibration waiting for valid sensor data...")
		return false
	}
	
	// Accumulate gyro readings
	s.gyroCalSum[0] += m.B1
	s.gyroCalSum[1] += m.B2  
	s.gyroCalSum[2] += m.B3
	s.gyroCalSamples++
	
	// Log progress every 50 samples
	if s.gyroCalSamples%50 == 0 {
		log.Printf("AHRS: Gyro calibration progress: %d/%d samples", s.gyroCalSamples, gyroCalDuration)
	}
	
	// Check if we have enough samples
	if s.gyroCalSamples >= gyroCalDuration {
		// Calculate average bias
		s.gyroBias[0] = s.gyroCalSum[0] / float64(s.gyroCalSamples)
		s.gyroBias[1] = s.gyroCalSum[1] / float64(s.gyroCalSamples)
		s.gyroBias[2] = s.gyroCalSum[2] / float64(s.gyroCalSamples)
		
		s.needsGyroCal = false  // Mark calibration complete
		
		log.Printf("AHRS: Gyro calibration complete!")
		log.Printf("AHRS: Gyro biases: X=%.3f Y=%.3f Z=%.3f deg/s", 
			s.gyroBias[0], s.gyroBias[1], s.gyroBias[2])
		
		return true
	}
	
	return false  // Still calibrating
}







































func (s *SimpleState) Compute(m *Measurement) {

	// First check if we need to calibrate gyro
	if s.needsGyroCal {
		gyroCalComplete := s.calibrateGyro(m)
		if !gyroCalComplete {
			return  // Don't run AHRS until gyro calibration is done
		}
	}

	if s.needsInitialization {
		s.init(m)
		log.Printf("INITIALIZATION BITCHESSSSSS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
	} else {
		s.Predict(m.T)
		s.Update(m)
	}
}

func (s *SimpleState) Predict(t float64) {
	return
}

































// Update performs the AHRSSimple AHRS computations.
// The idea behind the Simple AHRS algorithm is to use the GPS to compute what the accelerometer should show
// and then to create a rotation matrix to map the measured accelerometer vector onto this vector and the
// speed vector (GPS Track) onto the sensor x-axis.  Then the gyro is used to further improve this estimate.
// This is really a poor-man's sensor fusion algorithm.  The proper way to do this is with a Kalman Filter,
// but this approach is simpler and easier to debug, and should be good enough for most flight conditions.
//
// It is a step on the way to the full Kalman Filter implementation, a bit more obvious what's going on so
// the math and stratux integration can be more easily developed and debugged.

// SimpleDCM implements a Direction Cosine Matrix AHRS algorithm

func (s *SimpleState) Update(m *Measurement) {


	dt := m.T - s.T      //sensor delta-T
	dtw := m.TW - s.tW   //gps delta-T

	if dt > maxDT || dtw > maxDT {
		s.init(m)
		return
	}
	
	// Extract sensor data (assuming Measurement has these fields)
	// You'll need to adjust these field names to match the actual Measurement struct
	gx := (m.B1 - s.gyroBias[0]) * D2R // Gyro X (rad/s) - bias corrected
	gy := (m.B2 - s.gyroBias[1]) * D2R // Gyro Y (rad/s) - bias corrected  
	gz := (m.B3 - s.gyroBias[2]) * D2R // Gyro Z (rad/s) - bias corrected
	ax := m.A1 // Accel X (g)
	ay := m.A2 // Accel Y (g)
	az := m.A3 // Accel Z (g)
	mx := m.M2 // Mag X 
	my := m.M1 // Mag Y 
	mz := m.M3 // Mag Z 


	// Apply magnetometer calibration if enabled
    if s.magCalibrated {
        mx = (mx - s.magOffset[0]) * s.magScale[0]
        my = (my - s.magOffset[1]) * s.magScale[1]
        mz = (mz - s.magOffset[2]) * s.magScale[2]
    }
	
	
	s.roll = math.Atan2(ay, az)
	s.pitch = math.Atan2(-ax, math.Sqrt(ay*ay + az*az))

    m1 := mx * math.Cos(s.pitch) + my * math.Sin(s.roll);           
    m2 := mx * math.Sin(s.roll) * math.Sin(s.pitch) + my * math.Cos(s.roll) - mz * math.Sin(s.roll) * math.Cos(s.pitch);
    s.heading = math.Atan2(m2, m1);
	for s.heading < 0 {
		s.heading += 2 * Pi
	}
	for s.heading >= 2*Pi {
		s.heading -= 2 * Pi
	}
	
	s.F0, s.F1, s.F2, s.F3 = toQuaternion(s.roll, s.pitch, s.heading)
	s.calcRotationMatrices()

	//these were fixed to use body-fixed accelerations instead of inertial accelerations
	// Initialize Slip/Skid, Rate of Turn, and GLoad.
	s.slipSkid = math.Atan2(-m.A2, m.A3) * R2D
	s.turnRate = 0
	s.gLoad = m.A3 
	log.Printf("roll %f pitch %f yaw %f slip %f gload %f ax %f ay %f az %f gx %f gy %f gz %f mx %f my %f mz %f", s.roll*R2D, s.pitch*R2D, s.heading*R2D, s.slipSkid, s.gLoad, ax, ay, az, gx, gy, gz, mx, my, mz)
	
	updateLogMap(s, m, s.logMap)

	s.T = m.T
	s.tW = m.TW

}













// ResetGyroCal triggers a new gyro calibration
func (s *SimpleState) ResetGyroCal() {
	s.needsGyroCal = true
	s.gyroCalSamples = 0
	s.gyroCalSum = [3]float64{0, 0, 0}
	s.gyroBias = [3]float64{0, 0, 0}
	log.Printf("AHRS: Gyro calibration reset - keep IMU stationary!")
}






































 





// normalizeMatrix orthonormalizes the DCM matrix
func normalizeMatrix(in [3][3]float64) (out [3][3]float64) {
	// Get first two rows as vectors
	t0 := [3]float64{in[0][0], in[0][1], in[0][2]}
	t1 := [3]float64{in[1][0], in[1][1], in[1][2]}
	
	// Normalize first row
	norm := math.Sqrt(t0[0]*t0[0] + t0[1]*t0[1] + t0[2]*t0[2])
	if norm > 0 {
		for i := 0; i < 3; i++ {
			t0[i] /= norm
		}
	}
	
	// Make second row orthogonal to first
	dot := t0[0]*t1[0] + t0[1]*t1[1] + t0[2]*t1[2]
	for i := 0; i < 3; i++ {
		t1[i] -= dot * t0[i]
	}
	
	// Normalize second row
	norm = math.Sqrt(t1[0]*t1[0] + t1[1]*t1[1] + t1[2]*t1[2])
	if norm > 0 {
		for i := 0; i < 3; i++ {
			t1[i] /= norm
		}
	}
	
	// Third row is cross product of first two
	t2 := [3]float64{
		t0[1]*t1[2] - t0[2]*t1[1],
		t0[2]*t1[0] - t0[0]*t1[2],
		t0[0]*t1[1] - t0[1]*t1[0],
	}
	
	// Copy back to DCM
	for i := 0; i < 3; i++ {
		out[0][i] = t0[i]
		out[1][i] = t1[i]
		out[2][i] = t2[i]
	}
	return out
}


func updateLogMap(s *SimpleState, m *Measurement, p map[string]interface{}) {
	var simpleLogMap = map[string]func(s *SimpleState, m *Measurement) float64{
		"Ta":                func(s *SimpleState, m *Measurement) float64 { return s.T },
		"TWa":               func(s *SimpleState, m *Measurement) float64 { return s.tW },
		"Roll":              func(s *SimpleState, m *Measurement) float64 { return s.roll * D2R },
		"Pitch":             func(s *SimpleState, m *Measurement) float64 { return s.pitch * D2R },
		"Heading":           func(s *SimpleState, m *Measurement) float64 { return s.heading * D2R },
		"T":          func(s *SimpleState, m *Measurement) float64 { return m.T },
		"TW":         func(s *SimpleState, m *Measurement) float64 { return m.TW },
		"A1":         func(s *SimpleState, m *Measurement) float64 { return m.A1 },
		"A2":         func(s *SimpleState, m *Measurement) float64 { return m.A2 },
		"A3":         func(s *SimpleState, m *Measurement) float64 { return m.A3 },
		"B1":         func(s *SimpleState, m *Measurement) float64 { return m.B1 },
		"B2":         func(s *SimpleState, m *Measurement) float64 { return m.B2 },
		"B3":         func(s *SimpleState, m *Measurement) float64 { return m.B3 },
		"M1":         func(s *SimpleState, m *Measurement) float64 { return m.M1 },
		"M2":         func(s *SimpleState, m *Measurement) float64 { return m.M2 },
		"M3":         func(s *SimpleState, m *Measurement) float64 { return m.M3 },
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
    ["E0", "EGPS0", "EGyr0", "E0Actual", null],
    ["E1", "EGPS1", "EGyr1", "E1Actual", null],
    ["E2", "EGPS2", "EGyr2", "E2Actual", null],
    ["E3", "EGPS3", "EGyr3", "E3Actual", null],
    ["Z1", null, null, "Z1Actual", 0],
    ["Z2", null, null, "Z2Actual", 0],
    ["Z3", null, null, "Z3Actual", -1]
    ["C1", null, null, "C1Actual", 0],
    ["C2", null, null, "C2Actual", 0],
    ["C3", null, null, "C3Actual", 0]
    ["H1", null, null, "H1Actual", 0],
    ["H2", null, null, "H2Actual", 0],
    ["H3", null, null, "H3Actual", 0]
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
    ["B3", null, 0]
    ["M1", null, 0],
    ["M2", null, 0],
    ["M3", null, 0]
  ]
}`


// calcRotationMatrices populates the rotation matrices in the State based on
// the quaternions E and F
func (s *State) calcRotationMatrices() {
	// fij rotates sensor frame j component into aircraft frame i component
	// X_s = F*X_a*conj(F)
	s.f11 = (+s.F0*s.F0 + s.F1*s.F1 - s.F2*s.F2 - s.F3*s.F3)
	s.f12 = 2 * (-s.F0*s.F3 + s.F1*s.F2)
	s.f13 = 2 * (+s.F0*s.F2 + s.F3*s.F1)
	s.f21 = 2 * (+s.F0*s.F3 + s.F1*s.F2)
	s.f22 = (+s.F0*s.F0 - s.F1*s.F1 + s.F2*s.F2 - s.F3*s.F3)
	s.f23 = 2 * (-s.F0*s.F1 + s.F2*s.F3)
	s.f31 = 2 * (-s.F0*s.F2 + s.F3*s.F1)
	s.f32 = 2 * (+s.F0*s.F1 + s.F2*s.F3)
	s.f33 = (+s.F0*s.F0 - s.F1*s.F1 - s.F2*s.F2 + s.F3*s.F3)
}


// toQuaternion calculates the 0,1,2,3 components of the rotation quaternion
// corresponding to the Tait-Bryan angles phi, theta, psi
func toQuaternion(phi, theta, psi float64) (float64, float64, float64, float64) {
	theta = -theta        // We want positive theta to mean pitch up
	psi = math.Pi/2 - psi // We want psi to go N-E-S-W
	cphi := math.Cos(phi / 2)
	sphi := math.Sin(phi / 2)
	ctheta := math.Cos(theta / 2)
	stheta := math.Sin(theta / 2)
	cpsi := math.Cos(psi / 2)
	spsi := math.Sin(psi / 2)

	q0 := cphi*ctheta*cpsi + sphi*stheta*spsi
	q1 := sphi*ctheta*cpsi - cphi*stheta*spsi
	q2 := cphi*stheta*cpsi + sphi*ctheta*spsi
	q3 := cphi*ctheta*spsi - sphi*stheta*cpsi
	return q0, q1, q2, q3
}

// fromQuaternion calculates the Tait-Bryan angles phi, theta, psi corresponding to
// the quaternion
func fromQuaternion(q0, q1, q2, q3 float64) (phi float64, theta float64, psi float64) {
	phi = math.Atan2(2*(q0*q1+q2*q3), (q0*q0 - q1*q1 - q2*q2 + q3*q3))

	v := -2 * (q0*q2 - q3*q1) / (q0*q0 + q1*q1 + q2*q2 + q3*q3)
	if v >= 1 {
		theta = Pi / 2
	} else if v <= -1 {
		theta = -Pi / 2
	} else {
		theta = math.Asin(v)
	}
	psi = math.Pi/2 - math.Atan2(2*(q0*q3+q1*q2), (q0*q0+q1*q1-q2*q2-q3*q3))
	if psi < 0 {
		psi += 2 * math.Pi
	}
	return
}

// rotationMatrixToQuaternion computes the quaternion q corresponding to a rotation matrix r.
func rotationMatrixToQuaternion(r [3][3]float64) (q0, q1, q2, q3 float64) {
	q0 = math.Sqrt(1 + r[0][0] + r[1][1] + r[2][2])/2
	q1 = (r[2][1] - r[1][2])/(4*q0)
	q2 = (r[0][2] - r[2][0])/(4*q0)
	q3 = (r[1][0] - r[0][1])/(4*q0)
	return
}

// quaternionToRotationMatrix computes the rotation matrix r corresponding to a quaternion q.
func quaternionToRotationMatrix(q0, q1, q2, q3 float64) (r *[3][3]float64) {
	r = new([3][3]float64)
	r[0][0] = +q0*q0 + q1*q1 - q2*q2 - q3*q3
	r[0][1] = 2 * (-q0*q3 + q1*q2)
	r[0][2] = 2 * (+q0*q2 + q1*q3)
	r[1][0] = 2 * (+q0*q3 + q2*q1)
	r[1][1] = +q0*q0 - q1*q1 + q2*q2 - q3*q3
	r[1][2] = 2 * (-q0*q1 + q2*q3)
	r[2][0] = 2 * (-q0*q2 + q3*q1)
	r[2][1] = 2 * (+q0*q1 + q3*q2)
	r[2][2] = +q0*q0 - q1*q1 - q2*q2 + q3*q3
	return
}

// quaternionNormalize re-scales the input quaternion to unit norm.
func quaternionNormalize(q0, q1, q2, q3 float64) (r0, r1, r2, r3 float64) {
	qq := math.Sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3)
	r0 = q0 / qq
	r1 = q1 / qq
	r2 = q2 / qq
	r3 = q3 / qq
	return
}

















































































// Valid returns whether the current state is a valid estimate or if something went wrong in the calculation.
func (s *SimpleState) Valid() (ok bool) {
	return true
}


// Reset restarts the algorithm from scratch.
func (s *SimpleState) Reset() {
	s.needsInitialization = true
}

// RollPitchHeading returns the current attitude values as estimated by the Kalman algorithm.
func (s *SimpleState) RollPitchHeading() (roll float64, pitch float64, heading float64) {
	roll, pitch, heading = fromQuaternion(s.F0, s.F1, s.F2, s.F3)
	return
}


// MagHeading returns the magnetic heading in degrees.
func (s *SimpleState) MagHeading() (hdg float64) {

	return s.heading
}

// SlipSkid returns the slip/skid angle in degrees.
func (s *SimpleState) SlipSkid() (slipSkid float64) {
	return s.slipSkid
}

// RateOfTurn returns the turn rate in degrees per second.
func (s *SimpleState) RateOfTurn() (turnRate float64) {
	return 0
}

// GLoad returns the current G load, in G's.
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

