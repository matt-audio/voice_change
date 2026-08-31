# Changelog

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

### Verification

- Added standalone regression, full-rate, dependency, ASan, and UBSan tests.
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
