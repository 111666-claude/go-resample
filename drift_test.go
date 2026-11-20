package resample

import (
	"math"
	"math/rand"
	"testing"
	"time"
)

// This file verifies the timestamp-driven drift correction with simulated
// capture clocks. A simulated device produces blocks whose timestamps run at a
// fixed ppm offset from the output clock, plus per-block timestamp jitter; the
// DriftCorrector must estimate the offset and drive a Resampler so that the
// re-timed stream stays aligned with the target clock.

// driftSim is a simulated capture device on an independent clock.
type driftSim struct {
	nominalRate float64 // nominal device rate, Hz
	ppm         float64 // fixed device clock offset, ppm (>0 = fast)
	jitter      float64 // uniform timestamp jitter, seconds
	rng         *rand.Rand

	frames    int     // frames "captured" so far
	devTime   float64 // current device timestamp, seconds
	restartAt int     // inject a restart at this frame index, <0 disables
	restarted bool
}

// devicePeriod is the true inter-frame period of the drifting device clock.
// A fast clock (ppm > 0) stamps less device time per nominal frame period, so
// a fixed number of captured frames occupies fewer device seconds.
func (s *driftSim) devicePeriod() float64 {
	return (1.0 - s.ppm/1e6) / s.nominalRate
}

// next captures one block of up to maxFrames frames and returns the device
// timestamp at the END of the block and its frame count.
func (s *driftSim) next(maxFrames int) (t float64, n int) {
	n = maxFrames
	if s.restartAt >= 0 && !s.restarted && s.frames+n >= s.restartAt {
		// Restart: the new device time base jumps backwards.
		s.devTime = 0.5
		s.restarted = true
	}
	s.frames += n
	s.devTime += float64(n) * s.devicePeriod()
	t = s.devTime + (s.rng.Float64()*2-1)*s.jitter
	return t, n
}

// driftRun drives one full simulated recording through a corrector +
// resampler and reports the resulting timing behaviour.
type driftResult struct {
	outFrames       int
	discontinuities int
	diag            DriftDiagnostics
	firstRatio      float64
	events          map[DriftEvent]int
}

func runDriftSim(t *testing.T, sim *driftSim, seconds, blockFrames float64, cfg DriftConfig,
	conv Converter, inject func(block int, o *Observation)) driftResult {
	t.Helper()
	r, err := New(conv, 1)
	if err != nil {
		t.Fatal(err)
	}
	c, err := NewDriftCorrector(cfg)
	if err != nil {
		t.Fatal(err)
	}

	totalFrames := int(sim.nominalRate * seconds)
	in := rampInput(totalFrames)
	out := make([]float32, int(blockFrames*cfg.NominalRatio)+256)

	res := driftResult{events: map[DriftEvent]int{}}
	blockNo := 0
	inputPos := 0
	for consumed := 0; consumed < totalFrames; blockNo++ {
		n := int(math.Min(blockFrames, float64(totalFrames-consumed)))
		devT, _ := sim.next(n)
		inputPos += n

		o := Observation{
			DeviceTime:   devT,
			InputFrames:  inputPos,
			OutputFrames: res.outFrames,
		}
		if inject != nil {
			inject(blockNo, &o)
		}
		ratio, diag := c.Observe(o)
		if blockNo == 0 {
			res.firstRatio = ratio
		}
		res.events[diag.Event]++

		checkContinuity := func(gen int, eoi bool) {
			if conv != Linear || eoi {
				// The EndOfInput flush interpolates against zero-padded tail
				// samples, so its final steps are not part of the stream.
				return
			}
			for k := 1; k < gen; k++ {
				step := math.Abs(float64(out[k] - out[k-1]))
				if !(step > 0 && step < 2) {
					res.discontinuities++
				}
			}
		}

		d := &Data{
			In:           in[consumed : consumed+n],
			InputFrames:  n,
			Out:          out,
			OutputFrames: len(out),
			Ratio:        ratio,
			EndOfInput:   consumed+n == totalFrames,
		}
		if err := r.Process(d); err != nil {
			t.Fatal(err)
		}
		consumed += d.InputFramesUsed
		res.outFrames += d.OutputFramesGen
		checkContinuity(d.OutputFramesGen, d.EndOfInput)
		// Drain anything that did not fit, keeping the corrected ratio.
		for d.OutputFramesGen == d.OutputFrames {
			eoi := consumed == totalFrames
			d = &Data{Out: out, OutputFrames: len(out), Ratio: ratio, EndOfInput: eoi}
			if err := r.Process(d); err != nil {
				t.Fatal(err)
			}
			res.outFrames += d.OutputFramesGen
			checkContinuity(d.OutputFramesGen, eoi)
			if d.OutputFramesGen == 0 {
				break
			}
		}
	}
	res.diag = c.Diagnostics()
	return res
}

// TestDriftSimFastClock: group A, device clock fast (+120 ppm) with jitter.
func TestDriftSimFastClock(t *testing.T) {
	const seconds = 120
	sim := &driftSim{
		nominalRate: 48000, ppm: 120, jitter: 80e-6,
		rng: rand.New(rand.NewSource(1)), restartAt: -1,
	}
	cfg := DefaultDriftConfig(48000, 48000, 1)
	res := runDriftSim(t, sim, seconds, 960, cfg, Linear, nil)

	// Uncorrected error would be ppm * seconds = 14.4 ms.
	if baseline := math.Abs(sim.ppm) / 1e6 * seconds; baseline < 10e-3 {
		t.Fatalf("test setup: baseline error only %v s", baseline)
	}
	idealOut := float64(int(sim.nominalRate*seconds)) * cfg.NominalRatio
	if errS := math.Abs(float64(res.outFrames)-idealOut) / cfg.OutputRate; errS > 8e-3 {
		t.Errorf("final timing error %v s, want < 8 ms", errS)
	}
	if got := res.diag.DriftPPM; math.Abs(got-sim.ppm) > 8 {
		t.Errorf("estimated drift %v ppm, want ~%v ppm", got, sim.ppm)
	}
	if res.discontinuities != 0 {
		t.Errorf("output continuity broken: %d dropped/repeated frames", res.discontinuities)
	}
	if math.IsNaN(res.firstRatio) || math.IsInf(res.firstRatio, 0) {
		t.Fatalf("first ratio non-finite: %v", res.firstRatio)
	}
}

// TestDriftSimSlowClock: group B, device clock slow (-85 ppm), jitter,
// variable block sizes and a mid-stream device restart.
func TestDriftSimSlowClock(t *testing.T) {
	const seconds = 150
	sim := &driftSim{
		nominalRate: 44100, ppm: -85, jitter: 120e-6,
		rng:       rand.New(rand.NewSource(7)),
		restartAt: int(44100 * seconds / 2),
	}
	cfg := DefaultDriftConfig(44100, 48000, 48000.0/44100.0)

	// Variable block sizes, including occasional very small blocks: the
	// accumulated timing must not depend on the block size distribution.
	sizes := []float64{256, 512, 1024, 64, 441, 2048, 128}
	blockFrames := func(blockNo int) float64 { return sizes[blockNo%len(sizes)] }

	r, err := New(Linear, 1)
	if err != nil {
		t.Fatal(err)
	}
	c, err := NewDriftCorrector(cfg)
	if err != nil {
		t.Fatal(err)
	}
	totalFrames := int(sim.nominalRate * seconds)
	in := rampInput(totalFrames)
	out := make([]float32, 4096)
	outFrames := 0
	inputPos := 0
	reanchors := 0
	outAtRestart := 0
	var discontinuities int

	for consumed, blockNo := 0, 0; consumed < totalFrames; blockNo++ {
		n := int(math.Min(blockFrames(blockNo), float64(totalFrames-consumed)))
		devT, _ := sim.next(n)
		inputPos += n
		ratio, diag := c.Observe(Observation{
			DeviceTime: devT, InputFrames: inputPos, OutputFrames: outFrames,
		})
		if diag.Event == DriftEventReanchored {
			reanchors++
			outAtRestart = outFrames
		}
		d := &Data{
			In: in[consumed : consumed+n], InputFrames: n,
			Out: out, OutputFrames: len(out), Ratio: ratio,
			EndOfInput: consumed+n == totalFrames,
		}
		if err := r.Process(d); err != nil {
			t.Fatal(err)
		}
		consumed += d.InputFramesUsed
		outFrames += d.OutputFramesGen
		// Every adjacent step must stay in (0, 2) outside the EndOfInput
		// flush: a step outside that range would drop or repeat an input
		// frame. Ordinary fractional steps alternate around 1/ratio.
		if !d.EndOfInput {
			for k := 1; k < d.OutputFramesGen; k++ {
				step := math.Abs(float64(out[k] - out[k-1]))
				if !(step > 0 && step < 2) {
					discontinuities++
				}
			}
		}
		for d.OutputFramesGen == d.OutputFrames {
			d = &Data{Out: out, OutputFrames: len(out), Ratio: ratio, EndOfInput: consumed == totalFrames}
			if err := r.Process(d); err != nil {
				t.Fatal(err)
			}
			outFrames += d.OutputFramesGen
			if d.OutputFramesGen == 0 {
				break
			}
		}
	}
	diag := c.Diagnostics()
	if reanchors != 1 || diag.Reanchors != 1 {
		t.Errorf("restart handling: got %d reanchor events (counter %d), want 1", reanchors, diag.Reanchors)
	}
	if got := diag.DriftPPM; math.Abs(got-sim.ppm) > 12 {
		t.Errorf("post-restart drift estimate %v ppm, want ~%v ppm", got, sim.ppm)
	}
	// Timing error measured over the post-restart segment only: the restart
	// deliberately jumped the device time base, so compare the output produced
	// after the re-anchor against the ideal output for the frames captured
	// since the anchor.
	postFrames := totalFrames - sim.restartAt
	idealPostOut := float64(postFrames) * cfg.NominalRatio
	postOut := float64(outFrames - outAtRestart)
	if errS := math.Abs(postOut-idealPostOut) / cfg.OutputRate; errS > 20e-3 {
		t.Errorf("final timing error %v s, want < 20 ms", errS)
	}
	if discontinuities != 0 {
		t.Errorf("output continuity broken across variable blocks: %d dropped/repeated frames", discontinuities)
	}
}

// TestDriftBlockSizeInvariance: the cumulative correction must be independent
// of the block size. Running the same simulated clock with fixed block sizes of
// 128, 960 and 2048 frames must converge to the same final output position.
func TestDriftBlockSizeInvariance(t *testing.T) {
	const seconds = 60
	var lengths []int
	for _, block := range []float64{128, 960, 2048} {
		sim := &driftSim{
			nominalRate: 48000, ppm: 200, jitter: 50e-6,
			rng: rand.New(rand.NewSource(99)), restartAt: -1,
		}
		cfg := DefaultDriftConfig(48000, 48000, 1)
		res := runDriftSim(t, sim, seconds, block, cfg, SincFastest, nil)
		lengths = append(lengths, res.outFrames)
	}
	for i := 1; i < len(lengths); i++ {
		if d := math.Abs(float64(lengths[i] - lengths[0])); d > 4 {
			t.Errorf("block size changed accumulated output by %d frames (%v vs %v)", lengths[i]-lengths[0], lengths[i], lengths[0])
		}
	}
}

// TestDriftEstimatorConvergence checks the ppm estimate on a clean clock
// without jitter, where the regression must recover the exact offset.
func TestDriftEstimatorConvergence(t *testing.T) {
	for _, ppm := range []float64{-300, -50, 75, 250} {
		sim := &driftSim{
			nominalRate: 48000, ppm: ppm, jitter: 0,
			rng: rand.New(rand.NewSource(3)), restartAt: -1,
		}
		cfg := DefaultDriftConfig(48000, 48000, 1)
		res := runDriftSim(t, sim, 40, 480, cfg, Linear, nil)
		if got := res.diag.DriftPPM; math.Abs(got-ppm) > 3 {
			t.Errorf("ppm %v: estimated %v ppm, error %v", ppm, got, got-ppm)
		}
	}
}

// TestDriftNoDriftPassThrough: with a perfect clock the corrector must keep
// the ratio at the nominal value and a Linear resampler must produce exactly
// one output frame per input frame.
func TestDriftNoDriftPassThrough(t *testing.T) {
	sim := &driftSim{
		nominalRate: 48000, ppm: 0, jitter: 0,
		rng: rand.New(rand.NewSource(4)), restartAt: -1,
	}
	cfg := DefaultDriftConfig(48000, 48000, 1)
	res := runDriftSim(t, sim, 10, 480, cfg, Linear, nil)
	if math.Abs(res.diag.DriftPPM) > 1 {
		t.Errorf("zero-drift clock estimated at %v ppm", res.diag.DriftPPM)
	}
	if r := res.diag.Ratio; math.Abs(r-1) > 1e-6 {
		t.Errorf("ratio drifted to %v on a perfect clock", r)
	}
}

// TestDriftClampAndSlew checks the hard correction clamp and the per-second
// slew limit directly.
func TestDriftClampAndSlew(t *testing.T) {
	cfg := DefaultDriftConfig(48000, 48000, 1)
	cfg.MaxCorrection = 0.01
	cfg.SlewPerSecond = 0.001
	c, err := NewDriftCorrector(cfg)
	if err != nil {
		t.Fatal(err)
	}
	// Anchor.
	if ratio, _ := c.Observe(Observation{DeviceTime: 0, InputFrames: 480, OutputFrames: 0}); ratio != 1 {
		t.Fatalf("anchor ratio = %v, want 1", ratio)
	}
	// First block after anchor: 10 ms of device time, slew allows at most
	// 0.001 * 0.01 = 1e-5 of movement regardless of observed drift.
	ratio, d := c.Observe(Observation{DeviceTime: 0.01, InputFrames: 960, OutputFrames: 480})
	if d.State != DriftTracking {
		t.Errorf("state = %v, want Tracking", d.State)
	}
	if math.Abs(ratio-1) > cfg.SlewPerSecond*0.01+1e-12 {
		t.Errorf("slew violated: ratio moved by %v", ratio-1)
	}
	// A wildly drifting stream must never escape the hard clamp.
	for i := 0; i < 5000; i++ {
		dev := float64(i+2) * 0.01 * (1 + 0.5) // 50% "drift"
		ratio, _ = c.Observe(Observation{
			DeviceTime: dev, InputFrames: (i + 3) * 480,
			OutputFrames: (i + 2) * 480,
		})
		if math.Abs(ratio-1) > cfg.MaxCorrection+1e-12 {
			t.Fatalf("hard clamp violated at block %d: ratio %v", i, ratio)
		}
	}
}

// TestDriftSmoothnessNoLargeCorrections: short jitter bursts must not move the
// ratio by more than the slew budget of one block.
func TestDriftSmoothnessNoLargeCorrections(t *testing.T) {
	cfg := DefaultDriftConfig(48000, 48000, 1)
	c, err := NewDriftCorrector(cfg)
	if err != nil {
		t.Fatal(err)
	}
	c.Observe(Observation{DeviceTime: 0, InputFrames: 960, OutputFrames: 0})
	rng := rand.New(rand.NewSource(11))
	prev := 1.0
	prevDev := 0.0
	for i := 1; i < 2000; i++ {
		dev := float64(i)*0.02 + rng.NormFloat64()*200e-6
		ratio, _ := c.Observe(Observation{
			DeviceTime: dev, InputFrames: (i + 1) * 960, OutputFrames: i * 960,
		})
		// The slew budget is set by the actual (jittered) elapsed device time.
		slewDt := math.Min(dev-prevDev, cfg.RestartThreshold.Seconds())
		budget := cfg.SlewPerSecond * slewDt
		if math.Abs(ratio-prev) > budget+1e-15 {
			t.Fatalf("block %d: ratio jumped by %g, slew budget %g", i, ratio-prev, budget)
		}
		prev, prevDev = ratio, dev
	}
}

// TestDriftRecoveryEvents exercises every recovery path and checks the
// diagnostic counters and state transitions.
func TestDriftRecoveryEvents(t *testing.T) {
	cfg := DefaultDriftConfig(48000, 48000, 1)
	c, err := NewDriftCorrector(cfg)
	if err != nil {
		t.Fatal(err)
	}
	// Each block advances both clocks by 480 frames / 10 ms.
	in, out := 0, 0
	observe := func(devT float64, silent bool, level float64) DriftDiagnostics {
		in += 480
		_, d := c.Observe(Observation{
			DeviceTime: devT, InputFrames: in, OutputFrames: out,
			Silent: silent, BufferLevel: level,
		})
		out += 480
		return d
	}

	// Missing (NaN) and non-finite (+Inf) before any anchor: hold at nominal.
	if d := observe(math.NaN(), false, 0); d.Event != DriftEventMissing || d.State != DriftHold {
		t.Errorf("NaN: event=%v state=%v", d.Event, d.State)
	}
	if d := observe(math.Inf(1), false, 0); d.Event != DriftEventNonFinite || d.State != DriftHold {
		t.Errorf("Inf: event=%v state=%v", d.Event, d.State)
	}
	// Anchor at t=0.
	if d := observe(0, false, 0); d.State != DriftTracking {
		t.Fatalf("anchor: state=%v", d.State)
	}
	// Frozen clock: the next block reports the same device time again.
	if d := observe(0, false, 0); d.Event != DriftEventFrozen || d.State != DriftHold {
		t.Errorf("frozen: event=%v state=%v", d.Event, d.State)
	}
	// Small backwards step: reordered, ignored.
	if d := observe(-0.002, false, 0); d.Event != DriftEventReordered || d.State != DriftHold {
		t.Errorf("reorder: event=%v state=%v", d.Event, d.State)
	}
	// Large backwards step: restart/re-anchor.
	if d := observe(-5.0, false, 0); d.Event != DriftEventReanchored || d.State != DriftReanchored {
		t.Errorf("backwards restart: event=%v state=%v", d.Event, d.State)
	}
	// Large forward gap from the new base: restart/re-anchor.
	if d := observe(10.0, false, 0); d.Event != DriftEventReanchored {
		t.Errorf("forward restart: event=%v", d.Event)
	}
	// Counters for the first segment must all have fired before the reset.
	pre := c.Diagnostics()
	if pre.Missing == 0 || pre.NonFinite == 0 || pre.Frozen == 0 ||
		pre.Reordered == 0 || pre.Reanchors != 2 {
		t.Errorf("pre-reset recovery counters incomplete: %+v", pre)
	}

	// Re-anchor on a fresh corrector state for the clean-cadence segment.
	c.Reset()
	in, out = 0, 0
	observe(0, false, 0)
	// Silence freezes adaptation.
	before := c.Ratio()
	if d := observe(0.01, true, 0); d.Event != DriftEventSilent || d.State != DriftHold {
		t.Errorf("silence: event=%v state=%v", d.Event, d.State)
	}
	if c.Ratio() != before {
		t.Errorf("silence moved the ratio: %v -> %v", before, c.Ratio())
	}
	// Normal blocks at a steady 10 ms cadence to build a fit.
	for i := 2; i < 12; i++ {
		observe(float64(i)*0.01, false, 0)
	}
	baseRatio := c.Ratio()
	// A forward timestamp spike (50 ms ahead, still inside the restart
	// threshold) deviates from the fitted cadence and is rejected as an
	// outlier, not fit.
	if d := observe(12*0.01+0.05, false, 0); d.Event != DriftEventOutlier || d.State != DriftHold {
		t.Errorf("outlier: event=%v state=%v", d.Event, d.State)
	}
	if c.Ratio() != baseRatio {
		t.Errorf("outlier moved the ratio: %v -> %v", baseRatio, c.Ratio())
	}
	// Resume on the original cadence. The next timestamp is forward of the
	// rejected spike's predecessor (0.11 s) by exactly one 10 ms step.
	observe(13*0.01, false, 0)
	d := observe(14*0.01, false, 0.99)
	if d.Event != DriftEventOverfill || d.State != DriftTracking {
		t.Errorf("overfill: event=%v state=%v", d.Event, d.State)
	}
	if d.Ratio <= baseRatio {
		t.Errorf("overfill did not raise ratio: %v <= %v", d.Ratio, baseRatio)
	}
	if math.Abs(d.Ratio-1) > cfg.MaxCorrection+1e-12 {
		t.Errorf("overfill escaped hard clamp: %v", d.Ratio)
	}

	// After Reset only the second segment's counters are present.
	got := c.Diagnostics()
	if got.Silenced == 0 || got.Outliers == 0 || got.Overflow == 0 ||
		got.Missing != 0 || got.Reanchors != 0 || got.Frozen != 0 || got.Reordered != 0 {
		t.Errorf("post-reset recovery counters wrong: %+v", got)
	}
}

// TestDriftOverfillDrains: sustained overfill must push the ratio above unity
// and the boost must disappear once the buffer level returns below the
// high-water mark.
func TestDriftOverfillDrains(t *testing.T) {
	cfg := DefaultDriftConfig(48000, 48000, 1)
	c, err := NewDriftCorrector(cfg)
	if err != nil {
		t.Fatal(err)
	}
	c.Observe(Observation{DeviceTime: 0, InputFrames: 480, OutputFrames: 0})
	peak := 1.0
	for i := 1; i < 200; i++ {
		ratio, _ := c.Observe(Observation{
			DeviceTime: float64(i) * 0.01, InputFrames: (i + 1) * 480, OutputFrames: i * 480,
			BufferLevel: 0.97,
		})
		peak = math.Max(peak, ratio)
	}
	if peak <= 1 {
		t.Errorf("overfill never raised ratio above unity (peak %v)", peak)
	}
	// Once the buffer drains, the boost vanishes and the ratio returns to the
	// normal corrected range (no residual boost).
	ratio, _ := c.Observe(Observation{
		DeviceTime: 200 * 0.01, InputFrames: 201 * 480, OutputFrames: 200 * 480,
		BufferLevel: 0.2,
	})
	if math.Abs(ratio-1) > cfg.MaxCorrection {
		t.Errorf("ratio did not settle after drain: %v", ratio)
	}
}

// TestDriftReset clears all state and counters.
func TestDriftReset(t *testing.T) {
	cfg := DefaultDriftConfig(48000, 48000, 1)
	c, err := NewDriftCorrector(cfg)
	if err != nil {
		t.Fatal(err)
	}
	c.Observe(Observation{DeviceTime: 0, InputFrames: 480})
	c.Observe(Observation{DeviceTime: 0.01, InputFrames: 960, OutputFrames: 480})
	c.Observe(Observation{DeviceTime: math.NaN(), InputFrames: 1440, OutputFrames: 960})
	c.Reset()
	d := c.Diagnostics()
	if d.State != DriftInit || d.Points != 0 || d.Missing != 0 || d.Ratio != 1 {
		t.Errorf("Reset did not clear state: %+v", d)
	}
	// A fresh anchor behaves like a brand-new corrector.
	ratio, d2 := c.Observe(Observation{DeviceTime: 5, InputFrames: 480})
	if ratio != 1 || d2.State != DriftTracking || d2.DeviceTime != 0 {
		t.Errorf("post-Reset anchor wrong: ratio %v diag %+v", ratio, d2)
	}
}

// TestDriftConfigValidation covers constructor validation and defaults.
func TestDriftConfigValidation(t *testing.T) {
	if _, err := NewDriftCorrector(DefaultDriftConfig(0, 48000, 1)); err != ErrBadData {
		t.Errorf("zero input rate: got %v, want ErrBadData", err)
	}
	if _, err := NewDriftCorrector(DefaultDriftConfig(48000, -1, 1)); err != ErrBadData {
		t.Errorf("negative output rate: got %v, want ErrBadData", err)
	}
	bad := DefaultDriftConfig(48000, 48000, MaxRatio*2)
	if _, err := NewDriftCorrector(bad); err != ErrBadRatio {
		t.Errorf("bad nominal ratio: got %v, want ErrBadRatio", err)
	}
	bad = DefaultDriftConfig(48000, 48000, 1)
	bad.BufferHigh = 1.5
	if _, err := NewDriftCorrector(bad); err != ErrBadData {
		t.Errorf("bad buffer high: got %v, want ErrBadData", err)
	}
	// Zero tuning fields are replaced by defaults, and the resulting
	// corrector works.
	cfg := DriftConfig{NominalRatio: 1, InputRate: 48000, OutputRate: 48000}
	c, err := NewDriftCorrector(cfg)
	if err != nil {
		t.Fatalf("zero-value tuning fields: %v", err)
	}
	if c.cfg.Window != 30*time.Second || c.cfg.MaxPoints != 4096 || c.cfg.SlewPerSecond != 0.002 {
		t.Errorf("defaults not applied: %+v", c.cfg)
	}
	ratio, _ := c.Observe(Observation{DeviceTime: 0, InputFrames: 480})
	if ratio != 1 {
		t.Errorf("default corrector anchor ratio %v, want 1", ratio)
	}
}

// TestDriftProcessIntegration exercises the convenience Process wrapper,
// including its nil guards.
func TestDriftProcessIntegration(t *testing.T) {
	r, err := New(Linear, 1)
	if err != nil {
		t.Fatal(err)
	}
	c, err := NewDriftCorrector(DefaultDriftConfig(48000, 48000, 1))
	if err != nil {
		t.Fatal(err)
	}
	in := rampInput(960)
	out := make([]float32, 2048)
	d := &Data{In: in[:480], InputFrames: 480, Out: out, OutputFrames: len(out)}
	diag, err := c.Process(r, d, Observation{DeviceTime: 0, InputFrames: 480})
	if err != nil {
		t.Fatal(err)
	}
	if d.Ratio != diag.Ratio || d.OutputFramesGen == 0 {
		t.Errorf("Process wrapper did not drive Process: %+v", d)
	}
	if _, err := c.Process(nil, d, Observation{DeviceTime: 0.01}); err != ErrBadData {
		t.Errorf("nil resampler: got %v, want ErrBadData", err)
	}
	if _, err := c.Process(r, nil, Observation{DeviceTime: 0.01}); err != ErrBadData {
		t.Errorf("nil Data: got %v, want ErrBadData", err)
	}
}

// TestDriftString covers the diagnostic String methods.
func TestDriftString(t *testing.T) {
	if DriftState(99).String() != "Unknown" || DriftEvent(99).String() != "Unknown" {
		t.Error("unknown enum value did not return Unknown")
	}
	if DriftReanchored.String() != "Reanchored" || DriftEventOverfill.String() != "Overfill" {
		t.Error("known enum stringified incorrectly")
	}
}

// TestDriftZeroAlloc: once the regression ring is populated, Observe must not
// allocate, so it is safe to call from a real-time audio callback.
func TestDriftZeroAlloc(t *testing.T) {
	c, err := NewDriftCorrector(DefaultDriftConfig(48000, 48000, 1))
	if err != nil {
		t.Fatal(err)
	}
	c.Observe(Observation{DeviceTime: 0, InputFrames: 960})
	allocs := testing.AllocsPerRun(200, func() {
		for i := 1; i <= 32; i++ {
			c.Observe(Observation{
				DeviceTime:   float64(i) * 0.02,
				InputFrames:  (i + 1) * 960,
				OutputFrames: i * 960,
				BufferLevel:  0.5,
			})
		}
	})
	if allocs != 0 {
		t.Errorf("steady-state Observe allocated %v times per run, want 0", allocs)
	}
}
