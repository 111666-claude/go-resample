package resample

import (
	"math"
	"time"
)

// Drift correction for capture clocks that run slightly off the output clock.
//
// A USB audio capture device and the playback/output clock are independent
// oscillators: their rates typically differ by a few to a few hundred parts
// per million. Over a long recording that offset accumulates into audible
// audio/video (or audio/audio) desynchronisation. DriftCorrector estimates the
// rate offset from the device timestamps attached to each captured block and
// nudges the conversion Ratio of a [Resampler] just enough to keep the
// re-timed stream aligned with the target output clock.
//
// The estimator is a weighted least-squares fit of the number of frames the
// device has captured against its own timestamps. Critically, only input-side
// quantities enter the fit: the ratio the corrector itself produced changes
// the output frame counts, so feeding those back would make the estimator
// chase its own correction and cancel the drift estimate. The fitted slope is
// the true device rate, which is compared with the nominal rate to get the
// frequency offset in ppm. Weights are proportional to the time each
// observation spans, which makes the estimate independent of the block size.
//
// A phase-locked term pulls the accumulated phase error back to zero, and
// every ratio change is hard-clamped and slew-rate limited so short timestamp
// jitter never causes an audible correction.

// DriftState is the operating state reported in [DriftDiagnostics].
type DriftState int

const (
	// DriftInit is the state before the first valid timestamp has anchored the
	// estimator; the nominal ratio is passed through unchanged.
	DriftInit DriftState = iota
	// DriftTracking means timestamps are flowing and the estimator is adapting
	// the ratio.
	DriftTracking
	// DriftHold means adaptation is temporarily frozen (silence, invalid or
	// rejected timestamps); the last good ratio keeps being used.
	DriftHold
	// DriftReanchored means a discontinuity (backwards timestamp or large gap,
	// typically a device restart) was detected and the estimator restarted.
	DriftReanchored
)

// String returns the state name.
func (s DriftState) String() string {
	switch s {
	case DriftInit:
		return "Init"
	case DriftTracking:
		return "Tracking"
	case DriftHold:
		return "Hold"
	case DriftReanchored:
		return "Reanchored"
	default:
		return "Unknown"
	}
}

// DriftEvent classifies what happened on a single [DriftCorrector.Observe]
// call, as reported in [DriftDiagnostics.Event].
type DriftEvent int

const (
	// DriftEventNone: a normal observation was incorporated.
	DriftEventNone DriftEvent = iota
	// DriftEventReanchored: a backwards timestamp beyond the reorder tolerance
	// or a forward gap beyond the restart threshold restarted the estimator.
	DriftEventReanchored
	// DriftEventReordered: a small backwards timestamp was ignored.
	DriftEventReordered
	// DriftEventMissing: a NaN timestamp was ignored.
	DriftEventMissing
	// DriftEventNonFinite: an infinite timestamp was ignored.
	DriftEventNonFinite
	// DriftEventFrozen: a timestamp equal to the previous one (stalled device
	// clock) was ignored.
	DriftEventFrozen
	// DriftEventSilent: the block was flagged silent; adaptation was frozen.
	DriftEventSilent
	// DriftEventOverfill: the input buffer is over its high-water mark; a
	// bounded temporary ratio boost was applied to drain it.
	DriftEventOverfill
	// DriftEventOutlier: the timestamp deviated from the fitted clock too far
	// and was treated as jitter rather than incorporated.
	DriftEventOutlier
)

// String returns the event name.
func (e DriftEvent) String() string {
	switch e {
	case DriftEventNone:
		return "None"
	case DriftEventReanchored:
		return "Reanchored"
	case DriftEventReordered:
		return "Reordered"
	case DriftEventMissing:
		return "Missing"
	case DriftEventNonFinite:
		return "NonFinite"
	case DriftEventFrozen:
		return "Frozen"
	case DriftEventSilent:
		return "Silent"
	case DriftEventOverfill:
		return "Overfill"
	case DriftEventOutlier:
		return "Outlier"
	default:
		return "Unknown"
	}
}

// DriftConfig configures a [DriftCorrector]. Construct one with
// [DefaultDriftConfig] and override the fields you need.
type DriftConfig struct {
	// NominalRatio is output_rate / input_rate assuming perfectly matched
	// clocks, e.g. 48000.0/44100.0. The corrector multiplies it by a small
	// correction factor. Must lie in [MinRatio, MaxRatio].
	NominalRatio float64

	// InputRate and OutputRate are the nominal device and target clock rates
	// in hertz. Both must be positive.
	InputRate, OutputRate float64

	// Window is the time span of timestamps kept for the least-squares fit.
	// Longer windows reject jitter harder but react to real drift changes more
	// slowly. Default 30 s.
	Window time.Duration
	// MaxPoints bounds the number of retained observations; once reached the
	// oldest point is evicted. It makes the corrector allocation-free in
	// steady state. Default 4096.
	MaxPoints int

	// ReorderTolerance is the largest backwards timestamp step treated as a
	// harmless reordering (ignored) rather than a device restart (re-anchor).
	// Default 5 ms.
	ReorderTolerance time.Duration
	// RestartThreshold is the forward gap above which the device is assumed to
	// have restarted and the estimator re-anchors. Default 250 ms.
	RestartThreshold time.Duration

	// MaxCorrection bounds the deviation of the applied ratio from
	// NominalRatio, as a fraction (0.05 = 5%). Default 0.05.
	MaxCorrection float64
	// SlewPerSecond bounds how fast the correction factor may move per second
	// of device time (0.002 = 0.2%/s). A 100 ppm correction is then reached in
	// 50 ms, while a single jittery block cannot move the ratio audibly.
	// Default 0.002.
	SlewPerSecond float64
	// PhaseTimeConstant is the time constant of the phase-locked term that
	// drives accumulated phase error back to zero. Default 10 s.
	PhaseTimeConstant time.Duration
	// PhaseClamp bounds the phase-locked term as a fraction of the ratio
	// (0.01 = 1%), so a transient phase offset cannot force a large ratio.
	// Default 0.01.
	PhaseClamp float64

	// BufferHigh, when > 0, is the input-buffer fill fraction (0..1) above
	// which the buffer counts as overfull. An overfull buffer gets a bounded
	// temporary ratio boost (up to OverfillGain) to drain it. Default 0.9.
	BufferHigh float64
	// OverfillGain is the largest extra ratio fraction applied at a completely
	// overfull buffer (0.02 = 2%). Default 0.02.
	OverfillGain float64
}

// DefaultDriftConfig returns a configuration tuned for ordinary audio
// devices: a 30 s regression window, 5% hard correction limit, 0.2%/s slew and
// a 10 s phase-lock time constant. Rates and the nominal ratio must still be
// set on the returned value.
func DefaultDriftConfig(inputRate, outputRate, nominalRatio float64) DriftConfig {
	return DriftConfig{
		NominalRatio:      nominalRatio,
		InputRate:         inputRate,
		OutputRate:        outputRate,
		Window:            30 * time.Second,
		MaxPoints:         4096,
		ReorderTolerance:  5 * time.Millisecond,
		RestartThreshold:  250 * time.Millisecond,
		MaxCorrection:     0.05,
		SlewPerSecond:     0.002,
		PhaseTimeConstant: 10 * time.Second,
		PhaseClamp:        0.01,
		BufferHigh:        0.9,
		OverfillGain:      0.02,
	}
}

// Observation is one timestamped input block.
type Observation struct {
	// DeviceTime is the device timestamp of the block in seconds. The time
	// base is arbitrary (uptime, stream time, wall clock) but must be
	// monotonic while the device keeps running. NaN marks a missing timestamp
	// and ±Inf a non-finite one; both are handled as recovery events.
	DeviceTime float64
	// InputFrames is the cumulative number of input frames the device has
	// captured up to and including this block (a monotonically increasing
	// position, not a per-block length). It drives the drift estimate and is
	// measured on the input side, so the corrector's own ratio cannot feed
	// back into the estimate.
	InputFrames int
	// OutputFrames is the cumulative number of output frames produced so far,
	// i.e. the position on the target clock. It is used only for the phase
	// diagnostics, never for the drift estimate.
	OutputFrames int
	// Silent reports a silent block (for example a noise-gated or muted
	// device). Silent blocks keep the current ratio but do not feed the
	// estimator, since a stalled or muted device clock cannot be trusted.
	Silent bool
	// BufferLevel, when BufferHigh is configured, is the input-buffer fill
	// fraction in [0, 1].
	BufferLevel float64
}

// DriftDiagnostics is the per-block correction status. All durations are in
// seconds and all fractions are unitless; zero values mean "not yet known".
type DriftDiagnostics struct {
	// State is the current operating state.
	State DriftState
	// Event is what happened on this particular call.
	Event DriftEvent
	// Ratio is the ratio the caller should pass to Resampler.Process.
	Ratio float64
	// NominalRatio is the configured clock ratio without correction.
	NominalRatio float64
	// DriftPPM is the estimated device-clock offset, parts per million;
	// positive means the device clock runs fast.
	DriftPPM float64
	// PhaseSeconds is the current phase error, output time minus input time,
	// measured at the block boundary in output-clock seconds.
	PhaseSeconds float64
	// JitterSeconds is the EWMA magnitude of the timestamp residual against
	// the fitted clock.
	JitterSeconds float64
	// Points is the number of observations in the regression window.
	Points int

	// Cumulative recovery counters.
	Reanchors uint64
	Reordered uint64
	Missing   uint64
	NonFinite uint64
	Frozen    uint64
	Silenced  uint64
	Overflow  uint64
	Outliers  uint64

	// DeviceTime and OutputTime are the times underlying this observation,
	// shifted to the current anchor so they start near zero.
	DeviceTime float64
	OutputTime float64
}

// driftPoint is one weighted observation in the regression ring.
type driftPoint struct {
	x, y, w float64
}

// DriftCorrector estimates clock drift from timestamped input blocks and
// produces a smoothed, clamped conversion ratio. It is not safe for
// concurrent use; use one per stream, alongside one [Resampler].
type DriftCorrector struct {
	cfg DriftConfig

	inPeriod  float64 // 1 / InputRate
	outPeriod float64 // 1 / OutputRate

	ring    []driftPoint
	ringCap int
	head    int // index of the oldest point
	n       int // points currently stored

	// Weighted least-squares sums over the ring, in coordinates relative to
	// the anchor: x = device time, y = captured input frames.
	s0, sX, sY, sXX, sXY float64

	anchored    bool
	anchorT     float64 // device time of the anchor
	anchorIn    int     // cumulative input frames at the anchor
	anchorOut   int     // cumulative output frames at the anchor
	prevDevTime float64 // device time of the previous accepted observation

	driftPPM float64 // latest fitted offset, ppm
	jitter   float64 // EWMA residual magnitude, seconds
	phase    float64 // latest phase error, seconds

	mul float64 // applied correction factor (ratio = NominalRatio * mul)

	diag DriftDiagnostics
}

// NewDriftCorrector validates cfg and returns a corrector. Zero-valued tuning
// fields are replaced by the [DefaultDriftConfig] values.
func NewDriftCorrector(cfg DriftConfig) (*DriftCorrector, error) {
	if !IsValidRatio(cfg.NominalRatio) {
		return nil, ErrBadRatio
	}
	if cfg.InputRate <= 0 || cfg.OutputRate <= 0 ||
		math.IsNaN(cfg.InputRate) || math.IsInf(cfg.InputRate, 0) ||
		math.IsNaN(cfg.OutputRate) || math.IsInf(cfg.OutputRate, 0) {
		return nil, ErrBadData
	}
	def := DefaultDriftConfig(cfg.InputRate, cfg.OutputRate, cfg.NominalRatio)
	if cfg.Window <= 0 {
		cfg.Window = def.Window
	}
	if cfg.MaxPoints <= 0 {
		cfg.MaxPoints = def.MaxPoints
	}
	if cfg.ReorderTolerance <= 0 {
		cfg.ReorderTolerance = def.ReorderTolerance
	}
	if cfg.RestartThreshold <= 0 {
		cfg.RestartThreshold = def.RestartThreshold
	}
	if cfg.MaxCorrection <= 0 {
		cfg.MaxCorrection = def.MaxCorrection
	}
	if cfg.SlewPerSecond <= 0 {
		cfg.SlewPerSecond = def.SlewPerSecond
	}
	if cfg.PhaseTimeConstant <= 0 {
		cfg.PhaseTimeConstant = def.PhaseTimeConstant
	}
	if cfg.PhaseClamp <= 0 {
		cfg.PhaseClamp = def.PhaseClamp
	}
	if cfg.BufferHigh == 0 {
		cfg.BufferHigh = def.BufferHigh
	}
	if cfg.OverfillGain <= 0 {
		cfg.OverfillGain = def.OverfillGain
	}
	if cfg.BufferHigh < 0 || cfg.BufferHigh >= 1 || cfg.OverfillGain < 0 {
		return nil, ErrBadData
	}
	return &DriftCorrector{
		cfg:       cfg,
		inPeriod:  1.0 / cfg.InputRate,
		outPeriod: 1.0 / cfg.OutputRate,
		ring:      make([]driftPoint, cfg.MaxPoints),
		ringCap:   cfg.MaxPoints,
		mul:       1,
		diag:      DriftDiagnostics{State: DriftInit, Ratio: cfg.NominalRatio, NominalRatio: cfg.NominalRatio},
	}, nil
}

// Reset returns the corrector to its initial condition, dropping all
// observations and counters. The next [DriftCorrector.Observe] re-anchors.
func (c *DriftCorrector) Reset() {
	c.head, c.n = 0, 0
	c.s0, c.sX, c.sY, c.sXX, c.sXY = 0, 0, 0, 0, 0
	c.anchored = false
	c.anchorT, c.anchorIn, c.anchorOut, c.prevDevTime = 0, 0, 0, 0
	c.driftPPM, c.jitter, c.phase = 0, 0, 0
	c.mul = 1
	c.diag = DriftDiagnostics{State: DriftInit, Ratio: c.cfg.NominalRatio, NominalRatio: c.cfg.NominalRatio}
}

// Ratio returns the ratio that the most recent Observe call produced
// (NominalRatio before the first valid observation).
func (c *DriftCorrector) Ratio() float64 { return c.diag.Ratio }

// Diagnostics returns a copy of the latest diagnostics.
func (c *DriftCorrector) Diagnostics() DriftDiagnostics { return c.diag }

// Observe incorporates one timestamped block and returns the ratio to pass to
// [Resampler.Process], together with diagnostics. It never returns an error:
// malformed timestamps and discontinuities take the documented recovery paths
// and are reported through [DriftDiagnostics.Event] and the counters.
//
// Blocks with zero input frames (drain calls) need not be observed; call
// [DriftCorrector.Ratio] to keep using the current ratio.
func (c *DriftCorrector) Observe(o Observation) (float64, DriftDiagnostics) {
	d := c.diag
	d.NominalRatio = c.cfg.NominalRatio
	d.Event = DriftEventNone
	d.DriftPPM = c.driftPPM
	d.PhaseSeconds = c.phase
	d.JitterSeconds = c.jitter
	d.Points = c.n
	d.DeviceTime, d.OutputTime = 0, 0

	// --- Recovery: non-finite and missing timestamps ----------------------
	if math.IsNaN(o.DeviceTime) {
		d.Event, d.State, d.Missing = DriftEventMissing, DriftHold, d.Missing+1
		return c.hold(d)
	}
	if math.IsInf(o.DeviceTime, 0) {
		d.Event, d.State, d.NonFinite = DriftEventNonFinite, DriftHold, d.NonFinite+1
		return c.hold(d)
	}

	// --- First valid timestamp: anchor ------------------------------------
	if !c.anchored {
		c.anchor(o.DeviceTime, o.InputFrames, o.OutputFrames)
		d.State = DriftTracking
		return c.emit(d)
	}

	dt := o.DeviceTime - c.prevDevTime

	// --- Recovery: frozen and reordered clocks, device restart ------------
	if dt == 0 {
		d.Event, d.State, d.Frozen = DriftEventFrozen, DriftHold, d.Frozen+1
		return c.hold(d)
	}
	if dt < 0 {
		if -dt <= c.cfg.ReorderTolerance.Seconds() {
			// Small backwards step: a reordered USB packet, ignore it.
			d.Event, d.State, d.Reordered = DriftEventReordered, DriftHold, d.Reordered+1
			return c.hold(d)
		}
		// Large backwards step: the device restarted on an earlier time base.
		c.reanchor(o.DeviceTime, o.InputFrames, o.OutputFrames)
		d.Event, d.State, d.Reanchors = DriftEventReanchored, DriftReanchored, d.Reanchors+1
		return c.emit(d)
	}
	if dt > c.cfg.RestartThreshold.Seconds() {
		// Large forward gap: device restarted or paused; old history is stale.
		c.reanchor(o.DeviceTime, o.InputFrames, o.OutputFrames)
		d.Event, d.State, d.Reanchors = DriftEventReanchored, DriftReanchored, d.Reanchors+1
		return c.emit(d)
	}

	x := o.DeviceTime - c.anchorT
	y := float64(o.InputFrames - c.anchorIn)

	d.DeviceTime = x
	d.OutputTime = float64(o.OutputFrames-c.anchorOut) * c.outPeriod

	// --- Recovery: silence freezes adaptation -----------------------------
	if o.Silent {
		c.phase = d.OutputTime - y*c.inPeriod
		d.PhaseSeconds = c.phase
		d.Event, d.State, d.Silenced = DriftEventSilent, DriftHold, d.Silenced+1
		c.prevDevTime = o.DeviceTime
		return c.hold(d)
	}

	// --- Outlier rejection against the fitted clock -----------------------
	// Before the window has enough points the jitter estimate is untrusted,
	// so a generous fixed threshold applies (in frames, converted to seconds).
	threshold := 0.08 * c.cfg.InputRate
	if c.jitter > 0 {
		threshold = math.Max(8*c.jitter, 1e-3) * c.cfg.InputRate
	}
	if c.n >= 4 {
		b, a := c.fit()
		pred := a + b*x
		if math.Abs(y-pred) > threshold {
			// Do not advance prevDevTime: a rejected observation must not
			// rebase the clock, or the next in-cadence block would look like a
			// backwards jump and trigger a spurious restart.
			d.Event, d.State, d.Outliers = DriftEventOutlier, DriftHold, d.Outliers+1
			return c.hold(d)
		}
	}

	// --- Incorporate the weighted observation -----------------------------
	// Weight proportional to the time span the block represents, so changing
	// the block size does not change the regression (each second of audio
	// carries the same total weight).
	w := math.Max(dt, 1e-4)
	c.addPoint(driftPoint{x: x, y: y, w: w})
	c.evict(x)
	c.prevDevTime = o.DeviceTime

	if c.n >= 2 {
		b, a := c.fit()
		// Sanity: a fitted rate outside ±20% of nominal is not a believable
		// clock; keep the previous estimate rather than chase a bad fit.
		if b > c.cfg.InputRate*0.8 && b < c.cfg.InputRate*1.2 {
			c.driftPPM = (b/c.cfg.InputRate - 1) * 1e6
		}
		d.DriftPPM = c.driftPPM
		res := math.Abs(y-(a+b*x)) * c.inPeriod
		if c.jitter == 0 {
			c.jitter = res
		} else {
			c.jitter += 0.05 * (res - c.jitter)
		}
	}
	d.JitterSeconds = c.jitter
	d.Points = c.n

	// Phase error in output-clock seconds: how far the re-timed output
	// position is from the position the input frames nominally occupy.
	c.phase = d.OutputTime - y*c.inPeriod
	d.PhaseSeconds = c.phase

	// --- PLL correction: frequency term plus bounded phase term -----------
	// A fast device (positive ppm) must be resampled down: factor 1 - ppm.
	target := 1 - c.driftPPM/1e6
	phaseTerm := math.Max(-c.cfg.PhaseClamp, math.Min(c.cfg.PhaseClamp, c.phase/c.cfg.PhaseTimeConstant.Seconds()))
	target -= phaseTerm
	target = math.Max(1-c.cfg.MaxCorrection, math.Min(1+c.cfg.MaxCorrection, target))

	// Slew-limit over elapsed device time. Clamp the elapsed time to the
	// restart threshold so one long gap cannot cause one big jump.
	slewDt := math.Min(dt, c.cfg.RestartThreshold.Seconds())
	step := c.cfg.SlewPerSecond * slewDt
	c.mul = moveTowards(c.mul, target, step)

	// --- Recovery: buffer overfill gets a bounded temporary boost ---------
	if c.cfg.BufferHigh > 0 && o.BufferLevel > c.cfg.BufferHigh {
		overfill := (o.BufferLevel - c.cfg.BufferHigh) / (1 - c.cfg.BufferHigh)
		overfill = math.Max(0, math.Min(1, overfill))
		boost := overfill * c.cfg.OverfillGain
		// Applied after the slew and separately hard-clamped; it decays by
		// itself as the buffer level falls.
		mul := math.Max(1-c.cfg.MaxCorrection, math.Min(1+c.cfg.MaxCorrection, c.mul+boost))
		d.Ratio = c.clampRatio(c.cfg.NominalRatio * mul)
		d.Event, d.State, d.Overflow = DriftEventOverfill, DriftTracking, d.Overflow+1
		c.diag = d
		return d.Ratio, d
	}

	d.Ratio = c.clampRatio(c.cfg.NominalRatio * c.mul)
	d.State = DriftTracking
	c.diag = d
	return d.Ratio, d
}

// Process is a convenience wrapper: it observes o, writes the corrected ratio
// into d.Ratio and runs r.Process. The returned diagnostics describe the
// correction that was applied.
//
// Pass the same *[Data] you would hand to [Resampler.Process]; its Ratio field
// is overwritten. EndOfInput and the buffer fields are left untouched.
func (c *DriftCorrector) Process(r *Resampler, d *Data, o Observation) (DriftDiagnostics, error) {
	if r == nil || d == nil {
		return c.diag, ErrBadData
	}
	ratio, diag := c.Observe(o)
	d.Ratio = ratio
	if err := r.Process(d); err != nil {
		return diag, err
	}
	return diag, nil
}

// hold keeps the current ratio and state frozen for one call.
func (c *DriftCorrector) hold(d DriftDiagnostics) (float64, DriftDiagnostics) {
	d.Ratio = c.diag.Ratio
	c.diag = d
	return d.Ratio, d
}

// emit finalizes diagnostics after an anchor/re-anchor.
func (c *DriftCorrector) emit(d DriftDiagnostics) (float64, DriftDiagnostics) {
	d.Ratio = c.cfg.NominalRatio
	d.DriftPPM, d.PhaseSeconds, d.JitterSeconds = 0, 0, 0
	d.Points = 0
	c.diag = d
	return d.Ratio, d
}

// anchor starts a fresh time base at device time t with input/output positions.
func (c *DriftCorrector) anchor(t float64, in, out int) {
	c.anchored = true
	c.anchorT, c.anchorIn, c.anchorOut = t, in, out
	c.prevDevTime = t
	c.head, c.n = 0, 0
	c.s0, c.sX, c.sY, c.sXX, c.sXY = 0, 0, 0, 0, 0
	c.driftPPM, c.jitter, c.phase = 0, 0, 0
	c.mul = 1
}

// reanchor restarts the time base and leaves the state for the caller to label.
func (c *DriftCorrector) reanchor(t float64, in, out int) {
	c.anchor(t, in, out)
}

// addPoint inserts p into the ring and updates the regression sums.
func (c *DriftCorrector) addPoint(p driftPoint) {
	if c.n == c.ringCap {
		c.evictOldest()
	}
	idx := (c.head + c.n) % c.ringCap
	c.ring[idx] = p
	c.n++
	c.s0 += p.w
	c.sX += p.w * p.x
	c.sY += p.w * p.y
	c.sXX += p.w * p.x * p.x
	c.sXY += p.w * p.x * p.y
}

// evictOldest drops the oldest ring point and subtracts its contribution.
func (c *DriftCorrector) evictOldest() {
	p := c.ring[c.head]
	c.head = (c.head + 1) % c.ringCap
	c.n--
	c.s0 -= p.w
	c.sX -= p.w * p.x
	c.sY -= p.w * p.y
	c.sXX -= p.w * p.x * p.x
	c.sXY -= p.w * p.x * p.y
}

// evict drops points older than the regression window measured from nowX.
func (c *DriftCorrector) evict(nowX float64) {
	limit := nowX - c.cfg.Window.Seconds()
	for c.n > 0 && c.ring[c.head].x < limit {
		c.evictOldest()
	}
}

// fit returns the weighted least-squares slope and intercept of y on x.
func (c *DriftCorrector) fit() (slope, intercept float64) {
	den := c.s0*c.sXX - c.sX*c.sX
	if den <= 0 || c.s0 <= 0 {
		return 0, 0
	}
	slope = (c.s0*c.sXY - c.sX*c.sY) / den
	intercept = (c.sY - slope*c.sX) / c.s0
	return slope, intercept
}

// clampRatio keeps the corrected ratio inside the supported range.
func (c *DriftCorrector) clampRatio(ratio float64) float64 {
	if ratio < MinRatio {
		return MinRatio
	}
	if ratio > MaxRatio {
		return MaxRatio
	}
	return ratio
}

// moveTowards steps current towards target by at most step.
func moveTowards(current, target, step float64) float64 {
	if d := target - current; d > step {
		return current + step
	} else if d < -step {
		return current - step
	}
	return target
}
