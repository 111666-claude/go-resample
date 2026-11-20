// Package resample is a pure-Go (no cgo) audio sample-rate converter.
// See the NOTICE and LICENSE files for attribution.
//
// The package supports arbitrary and time-varying conversion ratios in the
// range [1/256, 256], where the ratio is defined as output_rate / input_rate.
// Audio is processed as interleaved float32 frames (one sample per channel per
// frame).
//
// # Converters
//
// Five converters trade quality against CPU cost:
//
//   - SincBestQuality   – widest bandwidth, deepest stopband, highest cost
//   - SincMediumQuality – transparent for most uses, moderate cost
//   - SincFastest       – good quality, low cost
//   - ZeroOrderHold     – trivial, nearest-sample hold
//   - Linear            – trivial, linear interpolation
//
// # Usage
//
// For a one-shot conversion of a whole buffer, use [Simple]:
//
//	out, err := resample.Simple(in, 48000.0/44100.0, 2, resample.SincMediumQuality)
//
// For streaming, create a [Resampler] and call [Resampler.Process] repeatedly,
// setting Data.EndOfInput on the final call.
//
// # Drift correction
//
// Independent capture and playback clocks (a USB sound card versus the output
// clock) typically differ by a few to a few hundred parts per million, which
// accumulates into audible audio/video desynchronisation on long recordings.
// [DriftCorrector] estimates that offset from the device timestamp of each
// captured block and drives the Resampler's Ratio to keep the re-timed stream
// aligned with the target clock:
//
//	c, _ := resample.NewDriftCorrector(resample.DefaultDriftConfig(inRate, outRate, ratio))
//	diag, err := c.Process(r, d, resample.Observation{
//	    DeviceTime: blockTimestamp, InputFrames: totalInputFrames,
//	})
//
// The estimate is a weighted least-squares fit over a sliding time window
// (block-size independent), combined with a phase-locked term, and every ratio
// change is hard-clamped and slew-rate limited so timestamp jitter never
// causes an audible correction. Backwards or missing timestamps, device
// restarts, silence, buffer overfill and non-finite times each take a defined
// recovery path reported through [DriftDiagnostics].
package resample
