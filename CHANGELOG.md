# Changelog

## 2026-09-01 - Wideband CPU optimization

### Changed

- Replaced the phase vocoder's full 1024-point complex transforms with packed
  real transforms backed by the existing 512-point FFT.
- Precomputed the fixed pitch-ratio bin mappings and skipped analysis bins that
  cannot contribute to each mode. Donald also skips an unused peak scan.
- Reused the project's sine lookup table during spectral synthesis.
- Replaced the 97-tap resampler with a symmetric 65-tap Kaiser filter. Paired
  symmetric samples halve its multiplications while retaining about -82 dB at
  the 12 kHz Nyquist limit.
- Precomputed shared FFT twiddles, inlined the biquad hot path, and removed
  integer modulo operations from the resampler ring cursors.

### Measured result

- Two five-run interleaved comparisons were run on the 30.013-second reference
  voice under different system loads. Male improved by 1.37-1.60x, Female by
  1.77-1.86x, and Donald by 1.38-1.67x.
- Across those batches, optimized Male/Female/Donald measured 1.00-1.26x
  Monster. They remained 1.40-1.87x Robot, whose YIN/PSOLA path is materially
  lighter than a four-times-overlapped wideband phase vocoder.
- The worst measured pitched-voice CPU hop was 0.157 ms, about 2.9% of the
  5.333 ms realtime deadline.
- Relative 4-12 kHz energy remains Male -28.4 dB, Female -18.3 dB, and Donald
  -18.1 dB; the optimization does not restore the old 4 kHz cutoff.

### Verification

- Verified target pitch, silence recovery, short inputs, full-rate processing,
  and 12.5-20 kHz anti-alias rejection.
- Verified real-FFT forward/inverse accuracy at 512, 1024, and 2048 samples.
- Processed the full reference input through all five modes with ASan and
  UBSan enabled.

## 2026-08-31 - Wideband realtime output

### Changed

- Raised the Male, Female, and Donald phase-vocoder sample rate from 8 kHz to
  24 kHz, moving the internal Nyquist limit from 4 kHz to 12 kHz.
- Replaced the 2.8 kHz resampling filter with a 97-tap, 10 kHz Kaiser filter
  that reaches about -81 dB at the new 12 kHz Nyquist limit.
- Increased the pitched-voice FFT from 512 to 1024 samples while retaining a
  four-times-overlapped 256-sample internal hop. A cached 1024-point
  bit-reversal table avoids rebuilding FFT indices in the realtime path.
- Added `PITCHED_VOICE_RATE_WIDEBAND`; the legacy numeric
  `PITCHED_VOICE_RATE_LOW_CPU` API remains accepted but now selects the same
  wideband path.
- The low-level `PITCHED_VOICE_RATE_FULL` option now shares the 1024/256
  phase-vocoder geometry. None of the five built-in modes uses this option,
  but direct users should expect changed startup latency and time resolution.
- Widened Robot's final low-pass from 6.5 kHz to 9.5 kHz.
- Widened Monster's LPC rolloff from 3.6-5.2 kHz to 6-9 kHz and its final
  low-pass from 5.6 kHz to 9 kHz. Monster remains intentionally dark.

### Measured result

- On the 30.013-second reference voice, relative 4-12 kHz energy changed from
  Robot -23.1 to -21.5 dB, Monster -33.9 to -29.6 dB, Male -77.8 to -28.1 dB,
  Female -61.6 to -18.3 dB, and Donald -50.0 to -18.0 dB.
- Median DSP CPU over 13 runs changed from 0.0512 to 0.0523 seconds for Robot,
  0.0772 to 0.0793 for Monster, 0.0562 to 0.1317 for Male, 0.0582 to 0.1411
  for Female, and 0.0552 to 0.1288 for Donald.
- The slowest measured CPU hop was 0.227 ms, about 4.3% of the 5.333 ms
  realtime deadline. Robot and Monster computational work is effectively
  unchanged; the extra cost is concentrated in the true wideband pitch shift.

### Verification

- Verified target pitch, silence preservation/recovery, short inputs, and
  same-file rejection with the standalone regression suite.
- Verified 12.5-20 kHz anti-alias rejection at two input phases.
- Verified both full-rate and wideband streaming paths with ASan and UBSan.
- Confirmed that the executable still links only to macOS `libSystem`.

## 2026-08-31 - Dependency-free realtime pitch voices

### Changed

- Replaced the Rubber Band pitch shifter with the project's own streaming
  phase vocoder using the existing FFT implementation.
- Removed the Rubber Band and libsamplerate build/runtime dependencies. The
  macOS executable now links only to the operating system `libSystem`.
- Kept realtime processing at 48 kHz with 256-sample input hops. Male, Female,
  and Donald use a fixed-size 512-sample phase-vocoder frame with a 128-sample
  internal hop and no processing-time allocation.
- Kept the voice pitches at Male -6 semitones, Female +6 semitones, and Donald
  +12 semitones.
- Retuned Donald for a slightly quieter, more cartoon-like sound:
  - makeup gain `1.5 -> 1.3`;
  - high-pass `180 -> 200 Hz`;
  - stronger, narrower nasal peaks near `780 Hz` and `1.95 kHz`;
  - slightly faster/deeper tremolo and chorus modulation.

### Verification before release

- Ran standalone regression, full-rate, dependency, ASan, and UBSan tests
  before packaging. Test sources are intentionally not part of this repository.
- Verified that all five modes preserve silence, output length, and realtime
  frame processing behavior.
- Measured Donald RMS on the 30.013-second reference input at `0.058352`, down
  from `0.063627` in the previous internal-pitch version.
- Measured the slowest Donald profile hop at about `125 us`, below the
  48 kHz / 256-sample realtime deadline of `5333 us`.

### Git checkpoints

- `e4e2fe7`: Rubber Band baseline imported into the local project history.
- `9cca0c3`: dependency-free implementation and verification hardening.
- `bf5def9`: quieter Donald cartoon retuning.
