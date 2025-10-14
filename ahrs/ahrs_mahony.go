package ahrs

import (
	"math"
)

const (
	Pi      = math.Pi
	R2D     = 180.0 / Pi
	D2R     = Pi / 180.0
	Invalid = 9_999_999.0

	minDT = 1e-6
	maxDT = 10.0 // seconds
)

/* ---------- Types wired to the rest of Stratux ---------- */

type State struct {
	T float64 // wallclock in ms (preserved for logging)
}

type Measurement struct {
	UValid, WValid, SValid, MValid bool
	A1, A2, A3                     float64 // accel (g)
	B1, B2, B3                     float64 // gyro (deg/s)
	M1, M2, M3                     float64 // mag (µT) (y, x, z)
	TW, TU, T                      float64 // timestamps (ms)
}

func NewMeasurement() *Measurement { return &Measurement{} }

/* ---------- Mahony-DCM filter ---------- */

type Mahony struct {
	State

	// Direction cosine matrix, world->body (R). Identity at init.
	// Columns are body axes expressed in world frame.
	R [3][3]float64

	// Integral bias term in BODY (rad/s).
	bias [3]float64

	// Gains
	KpAcc float64
	Ki    float64
	KpMag float64

	// book-keeping
	inited    bool
	last      Measurement
	lastEuler [3]float64 // roll, pitch, yaw (rad)
}

// NewAHRS returns a Mahony DCM filter with sane defaults.
func NewAHRS() *Mahony {
	m := &Mahony{
		KpAcc: 4.0,
		Ki:    0.05,
		KpMag: 16.0,
	}
	m.setIdentity()
	return m
}

// NewSimpleAHRS preserves the old name.
func NewSimpleAHRS() *Mahony { return NewAHRS() }

/* ---------- Public interface expected by sensors.go ---------- */

func (f *Mahony) Compute(meas *Measurement) {
	if meas == nil || !meas.SValid {
		return
	}

	// Time bookkeeping in seconds
	var dt float64
	if f.inited {
		dt = (meas.T - f.T) / 1000.0
		if dt <= minDT {
			return
		}
		if dt > maxDT {
			// stale; re-init with current sample
			f.initFrom(meas)
			return
		}
	} else {
		f.initFrom(meas)
		return
	}

	// BODY-frame sensors from Measurement
	ax, ay, az := meas.A1, meas.A2, meas.A3          // g
	gx, gy, gz := meas.B1*D2R, meas.B2*D2R, meas.B3*D2R // rad/s
	mx, my, mz := meas.M1, meas.M2, meas.M3          // note: your mapping (y,x,-z)->(M1,M2,M3)

	// --- Correction vector in BODY (proportional term) ---
	errx, erry, errz := f.correctionB(ax, ay, az, mx, my, mz, meas.MValid)

	// --- Integral (bias) ---
	if f.Ki > 0.0 {
		f.bias[0] += f.Ki * dt * errx
		f.bias[1] += f.Ki * dt * erry
		f.bias[2] += f.Ki * dt * errz
		// clamp integral to ~0.1 rad/s to avoid windup
		const bmax = 0.1
		f.bias[0] = clamp(f.bias[0], -bmax, bmax)
		f.bias[1] = clamp(f.bias[1], -bmax, bmax)
		f.bias[2] = clamp(f.bias[2], -bmax, bmax)
	}

	// --- Corrected angular rate in BODY ---
	ox := gx + f.KpAcc*errx - f.bias[0]
	oy := gy + f.KpAcc*erry - f.bias[1]
	oz := gz + f.KpAcc*errz - f.bias[2]

	// --- Integrate DCM: R_dot = R * [ω]_x
	var S [3][3]float64
	skew(ox, oy, oz, &S)

	// Forward Euler: R = R + R*S*dt
	var RS [3][3]float64
	mul3x3(&f.R, &S, &RS)
	for i := 0; i < 3; i++ {
		for j := 0; j < 3; j++ {
			f.R[i][j] += RS[i][j] * dt
		}
	}

	// Re-orthonormalize
	orthonormalize(&f.R)

	// Save for output/aux calcs
	f.T = meas.T
	f.last = *meas
	f.inited = true
}

func (f *Mahony) Valid() bool {
	if !f.inited {
		return false
	}
	// quick sanity: R columns unit length and finite
	for j := 0; j < 3; j++ {
		n := math.Sqrt(f.R[0][j]*f.R[0][j] + f.R[1][j]*f.R[1][j] + f.R[2][j]*f.R[2][j])
		if math.Abs(n-1) > 0.2 {
			return false
		}
	}
	return true
}

func (f *Mahony) Reset() {
	f.setIdentity()
	f.bias = [3]float64{}
	f.inited = false
	f.last = Measurement{}
	f.State = State{}
}

func (f *Mahony) RollPitchHeading() (roll, pitch, heading float64) {
	// Euler from DCM (world->body)
	// Using aerospace sequence (roll-X, pitch-Y, yaw-Z) from R
	// R(3,1) clamp for asin
	s := clamp(f.R[2][0], -1.0, 1.0)
	pitch = -math.Asin(s)
	roll = math.Atan2(f.R[2][1], f.R[2][2])
	heading = math.Atan2(f.R[1][0], f.R[0][0])

	// Normalize heading to [0, 2π)
	if heading < 0 {
		heading += 2 * Pi
	}
	return
}

func (f *Mahony) MagHeading() float64 {
	_, _, yaw := f.RollPitchHeading()
	return yaw
}

func (f *Mahony) SlipSkid() float64 {
	if !f.inited {
		return Invalid
	}
	ay, az := f.last.A2, f.last.A3
	return math.Atan2(-ay, az) * R2D
}

func (f *Mahony) RateOfTurn() float64 {
	if !f.inited {
		return Invalid
	}
	return f.last.B3 // deg/s
}

func (f *Mahony) GLoad() float64 {
	if !f.inited {
		return Invalid
	}
	return f.last.A3 // g
}

func (f *Mahony) GetState() *State { return &f.State }

func (f *Mahony) GetLogMap() map[string]interface{} {
	roll, pitch, yaw := f.RollPitchHeading()
	m := f.last
	return map[string]interface{}{
		"T": f.T,
		// Euler (deg)
		"Roll":    roll * R2D,
		"Pitch":   pitch * R2D,
		"Heading": yaw * R2D,
		// DCM
		"R11": f.R[0][0], "R12": f.R[0][1], "R13": f.R[0][2],
		"R21": f.R[1][0], "R22": f.R[1][1], "R23": f.R[1][2],
		"R31": f.R[2][0], "R32": f.R[2][1], "R33": f.R[2][2],
		// raw
		"A1": m.A1, "A2": m.A2, "A3": m.A3,
		"B1": m.B1, "B2": m.B2, "B3": m.B3,
		"M1": m.M1, "M2": m.M2, "M3": m.M3,
		// bias (rad/s)
		"BiasX": f.bias[0], "BiasY": f.bias[1], "BiasZ": f.bias[2],
		// gains
		"KpAcc": f.KpAcc, "Ki": f.Ki, "KpMag": f.KpMag,
		"dt": (f.T - f.last.T) / 1000.0,
	}
}

/* ---------- Initialization helpers ---------- */

func (f *Mahony) setIdentity() {
	f.R = [3][3]float64{
		{1, 0, 0},
		{0, 1, 0},
		{0, 0, 1},
	}
}

func (f *Mahony) initFrom(m *Measurement) {
	if m == nil {
    	f.setIdentity(); f.bias = [3]float64{}; f.inited = false; return
	}
	f.setIdentity()
	f.bias = [3]float64{}
	f.inited = true
	f.T = m.T
	f.last = *m

	// Seed attitude from accel + mag (if available)
	ax, ay, az := m.A1, m.A2, m.A3
	mx, my, mz := m.M2, m.M1, m.M3 // your mapping

	// Normalize accel to get gravity in BODY
	gN := math.Sqrt(ax*ax + ay*ay + az*az)
	if gN > 0.5 {
		ax /= gN
		ay /= gN
		az /= gN
		// BODY Z axis should align with +gravity direction in BODY
		// World +Z is down? We use world +Z up; accel measures +g along +Z body when sitting level.
		// Use accel as G_est = body Z wrt world; construct a consistent frame with mag for yaw.
		// Compute world X projection from mag
		if m.MValid {
			mN := math.Sqrt(mx*mx + my*my + mz*mz)
			if mN > 1e-6 {
				mx /= mN; my /= mN; mz /= mN
				// Remove vertical component from mag to get horizontal reference
				// G_est in BODY is (ax,ay,az); use it to remove vertical
				mVert := mx*ax + my*ay + mz*az
				mhx := mx - mVert*ax
				mhy := my - mVert*ay
				mhz := mz - mVert*az
				mhN := math.Sqrt(mhx*mhx + mhy*mhy + mhz*mhz)
				if mhN > 1e-6 {
					mhx /= mhN; mhy /= mhN; mhz /= mhN
					// Define BODY X to point where world +X projects in BODY: choose mh as +X
					// BODY Z is +G (ax,ay,az). BODY Y = Z × X (right-handed)
					bx := [3]float64{mhx, mhy, mhz}
					bz := [3]float64{ax, ay, az}
					var by [3]float64
					cross(bz, bx, &by)
					norm3(&bx)
					norm3(&by)
					norm3(&bz)
					// World->BODY DCM has columns = body axes in world; we have body axes in BODY!
					// To get R(world->body), we want how world axes look in BODY.
					// We constructed BODY axes (bx, by, bz) expressed in BODY; so R = [bx by bz]^T_world? 
					// Simpler: treat bx/by/bz as columns of R in WORLD coords.
					// Since these are BODY axes in BODY coords, the correct consistent DCM is identity at this step.
					// Instead, align body frame such that world axes map to our bx/by/bz in BODY by setting R = [bx by bz]^T
					f.R[0][0], f.R[1][0], f.R[2][0] = bx[0], bx[1], bx[2]
					f.R[0][1], f.R[1][1], f.R[2][1] = by[0], by[1], by[2]
					f.R[0][2], f.R[1][2], f.R[2][2] = bz[0], bz[1], bz[2]
					orthonormalize(&f.R)
					return
				}
			}
		}
		// Fallback: accel-only init (no yaw)
		// Choose X axis arbitrary horizontal
		hx := math.Sqrt(ay*ay + az*az)
		var bx, by, bz [3]float64
		if hx < 1e-6 {
			bx = [3]float64{1, 0, 0}
		} else {
			bx = [3]float64{0, az / hx, -ay / hx}
		}
		// Y = Z × X
		bz = [3]float64{ax, ay, az}
		cross(bz, bx, &by)
		norm3(&bx); norm3(&by); norm3(&bz)
		f.R[0][0], f.R[1][0], f.R[2][0] = bx[0], bx[1], bx[2]
		f.R[0][1], f.R[1][1], f.R[2][1] = by[0], by[1], by[2]
		f.R[0][2], f.R[1][2], f.R[2][2] = bz[0], bz[1], bz[2]
		orthonormalize(&f.R)
		return
	}

	// If accel unusable, just identity; filter will converge.
	f.setIdentity()
}

/* ---------- Core math from your Ada (ported) ---------- */

// Proportional correction in BODY
func (f *Mahony) correctionB(ax, ay, az, mx, my, mz float64, hasMag bool) (ex, ey, ez float64) {
	// World +Z up (0,0,1). G_est in BODY is R^T * Z_W -> column 3 of R^T is row 3 of R.
	// Compute R^T * [0,0,1] = third column of R^T = third row of R.
	gx := f.R[0][2]
	gy := f.R[1][2]
	gz := f.R[2][2]

	// Acc correction: align measured accel (gravity dir) with predicted gravity
	an := math.Sqrt(ax*ax + ay*ay + az*az)
	if an > 0.5 {
		ax /= an; ay /= an; az /= an
		// Err = a_meas × g_est
		ex += ay*gz - az*gy
		ey += az*gx - ax*gz
		ez += ax*gy - ay*gx
	}

	// Mag correction: use horizontal component only
	if hasMag {
		mn := math.Sqrt(mx*mx + my*my + mz*mz)
		if mn > 1e-6 {
			mx /= mn; my /= mn; mz /= mn
			// remove vertical component along g_est
			mvert := mx*gx + my*gy + mz*gz
			mhx := mx - mvert*gx
			mhy := my - mvert*gy
			mhz := mz - mvert*gz
			mhn := math.Sqrt(mhx*mhx + mhy*mhy + mhz*mhz)
			if mhn > 1e-6 {
				mhx /= mhn; mhy /= mhn; mhz /= mhn
				// world +X direction as seen in BODY is R^T * [1,0,0] = first row of R
				exw := f.R[0][0]
				eyw := f.R[1][0]
				ezw := f.R[2][0]
				// Err += wmag * (m_h × ex_world_in_body)
				wmag := f.KpMag / math.Max(f.KpAcc, 1e-6)
				ex += wmag * (mhy*ezw - mhz*eyw)
				ey += wmag * (mhz*exw - mhx*ezw)
				ez += wmag * (mhx*eyw - mhy*exw)
			}
		}
	}

	return
}

/* ---------- Linear algebra helpers ---------- */

func clamp(x, a, b float64) float64 {
	if x < a {
		return a
	}
	if x > b {
		return b
	}
	return x
}

func skew(wx, wy, wz float64, S *[3][3]float64) {
	// [ 0  -wz  wy]
	// [ wz  0  -wx]
	// [-wy wx   0 ]
	S[0][0], S[0][1], S[0][2] = 0, -wz, wy
	S[1][0], S[1][1], S[1][2] = wz, 0, -wx
	S[2][0], S[2][1], S[2][2] = -wy, wx, 0
}

func mul3x3(A, B, C *[3][3]float64) {
	for i := 0; i < 3; i++ {
		for j := 0; j < 3; j++ {
			C[i][j] = A[i][0]*B[0][j] + A[i][1]*B[1][j] + A[i][2]*B[2][j]
		}
	}
}

func cross(a, b [3]float64, c *[3]float64) {
	c[0] = a[1]*b[2] - a[2]*b[1]
	c[1] = a[2]*b[0] - a[0]*b[2]
	c[2] = a[0]*b[1] - a[1]*b[0]
}

func norm3(v *[3]float64) float64 {
	n := math.Sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2])
	if n > 1e-9 {
		in := 1.0 / n
		v[0] *= in
		v[1] *= in
		v[2] *= in
	} else {
		v[0], v[1], v[2] = 0, 0, 0
	}
	return n
}

func orthonormalize(R *[3][3]float64) {
	// Columns as vectors
	c0 := [3]float64{R[0][0], R[1][0], R[2][0]}
	c1 := [3]float64{R[0][1], R[1][1], R[2][1]}

	// 1) normalize c0
	if norm3(&c0) == 0 {
		c0 = [3]float64{1, 0, 0}
	}

	// 2) orthogonalize & normalize c1
	proj := c0[0]*c1[0] + c0[1]*c1[1] + c0[2]*c1[2]
	c1[0] -= proj * c0[0]
	c1[1] -= proj * c0[1]
	c1[2] -= proj * c0[2]
	if norm3(&c1) == 0 {
		c1 = [3]float64{0, 1, 0}
	}

	// 3) c2 = c0 × c1
	var c2 [3]float64
	cross(c0, c1, &c2)
	norm3(&c2)
	if c2 == ([3]float64{0, 0, 0}) {
		c2 = [3]float64{0, 0, 1}
	}

	R[0][0], R[1][0], R[2][0] = c0[0], c0[1], c0[2]
	R[0][1], R[1][1], R[2][1] = c1[0], c1[1], c1[2]
	R[0][2], R[1][2], R[2][2] = c2[0], c2[1], c2[2]
}


// Level resets attitude to "current level" using accel (+mag if valid).
func (f *Mahony) Level() {
    if !f.inited {
        return
    }
    // Re-seed from last measurement (accel/mag)
    f.initFrom(&f.last)
}

// Cage = Level+zero integral bias (gyro drift integrator).
func (f *Mahony) Cage() {
    f.Level()
    f.bias = [3]float64{}
}

// CalibrateMag is a stub hook; for now just re-level with mag and clear bias.
// (You can later replace with a real hard/soft-iron calibration routine.)
func (f *Mahony) CalibrateMag() { f.Cage() }

// Optional convenience:
func (f *Mahony) SetGains(kpAcc, ki, kpMag float64) { f.KpAcc, f.Ki, f.KpMag = kpAcc, ki, kpMag }


