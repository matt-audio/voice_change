# voice_change_

`voice_change_` is a standalone realtime voice-effects project. It provides
Robot, Monster, Male, Female, and Donald modes. The runtime DSP implementation
is entirely contained in `src/` and processes 256-sample input hops at 48 kHz.

Male, Female, and Donald use the project's own streaming phase vocoder, FFT,
sample-rate conversion, filters, chorus, and dynamics. Robot and Monster use
the project's own YIN, PSOLA, LPC, filters, and dynamics. No third-party audio
library is required.

## Build

Requirements are a C11 compiler, `make`, and the platform C math library:

```sh
make clean all
```

On macOS the executable links only to the operating system `libSystem`.
The optional regression tests use Python 3's standard library and standard OS
binary-inspection tools; neither is part of the delivered runtime.

## Run

```sh
./voice_fx male input.wav output.wav
./voice_fx female input.wav output.wav
./voice_fx donald input.wav output.wav
./voice_fx robot input.wav output.wav
./voice_fx monster input.wav output.wav
```

Input must be mono, 16-bit PCM WAV at 48 kHz. Male shifts -6 semitones, Female
shifts +6 semitones, and Donald shifts +12 semitones with additional cartoon
chorus, EQ, tremolo, and dynamics.

## Verify

```sh
make test
make sanitize-test
```

The test target includes an OS-library-only dynamic dependency check.

Release history and measured changes are recorded in [CHANGELOG.md](CHANGELOG.md).
