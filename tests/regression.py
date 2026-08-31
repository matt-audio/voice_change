#!/usr/bin/env python3

import array
import hashlib
import math
import os
import pathlib
import subprocess
import sys
import tempfile
import wave


SAMPLE_RATE = 48000
ALL_MODES = ("robot", "monster", "male", "female", "donald")


def write_wav(path, samples):
    pcm = array.array("h", (round(max(-1.0, min(1.0, sample)) * 32767.0)
                            for sample in samples))
    with wave.open(str(path), "wb") as output:
        output.setnchannels(1)
        output.setsampwidth(2)
        output.setframerate(SAMPLE_RATE)
        output.writeframes(pcm.tobytes())


def read_wav(path):
    with wave.open(str(path), "rb") as input_file:
        assert input_file.getnchannels() == 1
        assert input_file.getsampwidth() == 2
        assert input_file.getframerate() == SAMPLE_RATE
        samples = array.array("h")
        samples.frombytes(input_file.readframes(input_file.getnframes()))
    if sys.byteorder != "little":
        samples.byteswap()
    return [sample / 32768.0 for sample in samples]


def read_pcm(path):
    with wave.open(str(path), "rb") as input_file:
        return input_file.readframes(input_file.getnframes())


def run(binary, *arguments):
    return subprocess.run([str(binary), *map(str, arguments)],
                          stdout=subprocess.DEVNULL,
                          stderr=subprocess.DEVNULL,
                          check=False)


def rms(samples):
    return math.sqrt(sum(sample * sample for sample in samples) / len(samples))


def dominant_frequency(samples, minimum_hz, maximum_hz):
    mean = sum(samples) / len(samples)
    centered = [sample - mean for sample in samples]
    minimum_lag = round(SAMPLE_RATE / maximum_hz)
    maximum_lag = round(SAMPLE_RATE / minimum_hz)

    def correlation(lag):
        return sum(centered[i] * centered[i + lag]
                   for i in range(len(centered) - lag))

    best_lag = max(range(minimum_lag, maximum_lag + 1), key=correlation)
    return SAMPLE_RATE / best_lag


def sinusoid_residual(samples, frequency):
    mean = sum(samples) / len(samples)
    centered = [sample - mean for sample in samples]
    cosine = 2.0 * sum(
        sample * math.cos(2.0 * math.pi * frequency * i / SAMPLE_RATE)
        for i, sample in enumerate(centered)) / len(centered)
    sine = 2.0 * sum(
        sample * math.sin(2.0 * math.pi * frequency * i / SAMPLE_RATE)
        for i, sample in enumerate(centered)) / len(centered)
    residual = [
        sample - cosine * math.cos(
            2.0 * math.pi * frequency * i / SAMPLE_RATE) -
        sine * math.sin(2.0 * math.pi * frequency * i / SAMPLE_RATE)
        for i, sample in enumerate(centered)
    ]
    return rms(residual) / rms(centered)


def test_short_and_silent(binary, workdir):
    for name, samples in (("empty", []), ("short", [0.1] * 480),
                          ("silence", [0.0] * SAMPLE_RATE)):
        source = workdir / f"{name}.wav"
        write_wav(source, samples)
        for mode in ALL_MODES:
            output = workdir / f"{name}_{mode}.wav"
            assert run(binary, mode, source, output).returncode == 0
            rendered = read_wav(output)
            assert len(rendered) == len(samples), f"{mode} changed WAV length"
            if name == "silence":
                assert not any(read_pcm(output)), f"{mode} generated sound"


def test_same_file_rejected(binary, workdir):
    source = workdir / "same.wav"
    write_wav(source, [0.1] * 2048)
    original = source.read_bytes()
    assert run(binary, "robot", source, source).returncode != 0
    assert source.read_bytes() == original
    alias = workdir / "same_alias.wav"
    os.link(source, alias)
    assert run(binary, "monster", source, alias).returncode != 0
    assert source.read_bytes() == original


def test_relative_pitch_modes(binary, workdir):
    source = workdir / "pitch.wav"
    write_wav(source, [0.18 * math.sin(2.0 * math.pi * 140.0 * i /
                                      SAMPLE_RATE)
                       for i in range(SAMPLE_RATE)])
    expected = {
        "male": (140.0 * 2.0 ** (-6.0 / 12.0), 80.0, 125.0, 0.05),
        "female": (140.0 * 2.0 ** (6.0 / 12.0), 165.0, 235.0, 0.05),
        "donald": (280.0, 235.0, 330.0, 0.15),
    }
    for mode, (target, low, high, maximum_residual) in expected.items():
        output = workdir / f"pitch_{mode}.wav"
        assert run(binary, mode, source, output).returncode == 0
        stable = read_wav(output)[SAMPLE_RATE // 3:2 * SAMPLE_RATE // 3]
        measured = dominant_frequency(stable, low, high)
        assert abs(measured - target) / target < 0.025, (
            f"{mode}: measured {measured:.2f} Hz, expected {target:.2f} Hz")
        residual = sinusoid_residual(stable, target)
        assert residual < maximum_residual, (
            f"{mode}: {residual:.1%} non-pitch energy")


def test_pitched_silence_recovery(binary, workdir):
    source = workdir / "resume.wav"
    samples = []
    phase = 0.0
    step = 2.0 * math.pi * 140.0 / SAMPLE_RATE
    for seconds, voiced in ((0.5, True), (0.3, False), (0.5, True)):
        for _ in range(round(seconds * SAMPLE_RATE)):
            samples.append(0.18 * math.sin(phase) if voiced else 0.0)
            phase += step
    write_wav(source, samples)
    for mode in ("male", "female", "donald"):
        output = workdir / f"resume_{mode}.wav"
        assert run(binary, mode, source, output).returncode == 0
        rendered = read_wav(output)
        assert rms(rendered[round(0.62 * SAMPLE_RATE):
                            round(0.70 * SAMPLE_RATE)]) < 1e-4
        assert rms(rendered[round(0.88 * SAMPLE_RATE):
                            round(1.08 * SAMPLE_RATE)]) > 0.005


def test_resampler_alias_rejection(binary, workdir):
    for frequency in (4100.0, 4500.0, 7000.0):
        for phase in (0.0, 0.37):
            source = workdir / f"alias_{frequency}_{phase}.wav"
            write_wav(source, [0.2 * math.sin(
                2.0 * math.pi * frequency * i / SAMPLE_RATE + phase)
                for i in range(SAMPLE_RATE)])
            for mode in ("male", "female", "donald"):
                output = workdir / f"alias_{mode}_{frequency}_{phase}.wav"
                assert run(binary, mode, source, output).returncode == 0
                stable = read_wav(output)[SAMPLE_RATE // 3:
                                          2 * SAMPLE_RATE // 3]
                assert rms(stable) < 5e-4, f"{mode} aliased {frequency:g} Hz"


def test_robot_monster_golden(binary, workdir):
    source = workdir / "golden.wav"
    period = 384
    write_wav(source, [0.18 if i % period < period // 2 else -0.18
                       for i in range(8192)])
    expected = (
        ("robot", None,
         "4d4aa85fd316ee04eba66ff22a12ad1673eba2ecb3f0c0706fd840c7451c9d05"),
        ("monster", None,
         "8a62df33c139c83ab3875e41319d7f16caad343e1172ccdb3bc73d8d4a36d1cd"),
        ("monster", "440",
         "f123e80510ff0ad13ffd8b8c2adcbb7d30fdc5a9bab51c64b489dbf4e08895e7"),
    )
    for mode, target, expected_hash in expected:
        output = workdir / f"golden_{mode}_{target}.wav"
        extra = (target,) if target else ()
        assert run(binary, mode, source, output, *extra).returncode == 0
        actual = hashlib.sha256(read_pcm(output)).hexdigest()
        assert actual == expected_hash, f"{mode} PCM output changed"


def main():
    if len(sys.argv) != 2:
        raise SystemExit(f"Usage: {sys.argv[0]} <voice_fx>")
    binary = pathlib.Path(sys.argv[1]).resolve()
    with tempfile.TemporaryDirectory(prefix="voice_change_test.") as directory:
        workdir = pathlib.Path(directory)
        test_short_and_silent(binary, workdir)
        test_same_file_rejected(binary, workdir)
        test_relative_pitch_modes(binary, workdir)
        test_pitched_silence_recovery(binary, workdir)
        test_resampler_alias_rejection(binary, workdir)
        test_robot_monster_golden(binary, workdir)
    print("All regression tests passed")


if __name__ == "__main__":
    main()
