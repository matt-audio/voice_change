#!/usr/bin/env python3
"""Unified Traditional DSP Voice Effects Engine for Mono WAV Files.

Combines FX transformers (Donald, Robot, Monster) and Gender Swapping
(To Male, To Female) into a single streamlined CLI script using Rubber Band,
FFmpeg, and Praat.
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
import tempfile
import wave
from pathlib import Path

ROOT = Path(__file__).resolve().parent
DEFAULT_INPUTS = (ROOT / "raw.wav",)
OUTPUT_DIR = ROOT / "outputs"
PRAAT_SCRIPT = ROOT / "flatten_pitch.praat"

# Pitch Target Constants
ROBOT_TARGET_HZ = 52.0
MONSTER_FLATTEN_HZ = 44.78174593  # D#2 at A=440 Hz

# --- Common Utilities ---


def run(command: list[str]) -> None:
    print("+", " ".join(command))
    subprocess.run(command, check=True)


def duration_seconds(path: Path) -> float:
    with wave.open(str(path), "rb") as wav:
        return wav.getnframes() / wav.getframerate()


def require_tools(check_praat: bool = True) -> None:
    tools = ["rubberband", "ffmpeg"]
    if check_praat:
        tools.append("praat")
    missing = [name for name in tools if shutil.which(name) is None]
    if missing:
        raise RuntimeError("Missing required external tools: " + ", ".join(missing))


# --- Core DSP Primitive Operations ---


def pitch_shift(
    source: Path,
    destination: Path,
    semitones: float,
    preserve_formants: bool = False,
) -> None:
    command = ["rubberband", "-3", "-q"]
    if preserve_formants:
        command.append("-F")
    command.extend([
        f"-p{semitones:g}",
        str(source),
        str(destination),
    ])
    run(command)


def flatten_f0(source: Path, destination: Path, target_hz: float) -> None:
    if not PRAAT_SCRIPT.exists():
        raise FileNotFoundError(f"Praat script not found: {PRAAT_SCRIPT}")
    run([
        "praat",
        "--FULL-TRUST",
        "--utf8",
        "--run",
        str(PRAAT_SCRIPT),
        str(source),
        str(destination),
        f"{target_hz:g}",
    ])


def ffmpeg_filter(source: Path, destination: Path, filter_graph: str) -> None:
    run([
        "ffmpeg",
        "-y",
        "-hide_banner",
        "-loglevel",
        "error",
        "-i",
        str(source),
        "-af",
        filter_graph,
        "-ar",
        "48000",
        "-ac",
        "1",
        "-c:a",
        "pcm_s16le",
        str(destination),
    ])


# --- Voice Effect Generators ---


def make_donald(source: Path, destination: Path, temp: Path) -> None:
    """Donald Duck voice (High pitch, formants scaled, frequency boost + tremolo)."""
    shifted = temp / "donald_pitch.wav"
    pitch_shift(source, shifted, 11.0)
    filters = (
        "highpass=f=200,"
        "equalizer=f=1000:width_type=q:width=1.2:g=4.0,"
        "equalizer=f=2500:width_type=q:width=1.5:g=7.0,"
        "equalizer=f=4000:width_type=q:width=1.0:g=3.0,"
        "tremolo=f=10:d=0.35,"
        "acompressor=threshold=0.12:ratio=6:attack=2:release=60:makeup=2,"
        "asoftclip=type=tanh:threshold=0.70:param=0.35:oversample=4,"
        "alimiter=limit=0.92,"
        "loudnorm=I=-16:TP=-1.5:LRA=7"
    )
    ffmpeg_filter(shifted, destination, filters)


def make_robot(source: Path, destination: Path, temp: Path) -> None:
    """Robot voice (Flattened F0 via Praat, ring modulation + flanger)."""
    flattened = temp / "robot_f0_flattened.wav"
    flatten_f0(source, flattened, ROBOT_TARGET_HZ)
    filters = (
        "asplit=2[dry][voice];"
        "[voice]aeval=exprs='val(0)*sin(2*PI*52*t)'[ring];"
        "[dry]volume=0.02[dry_quiet];"
        "[ring]volume=0.98[ring_quiet];"
        "[dry_quiet][ring_quiet]amix=inputs=2:duration=first:normalize=0,"
        "flanger=delay=5:depth=3:regen=42:width=48:speed=0.22,"
        "highpass=f=45,"
        "equalizer=f=180:width_type=q:width=0.9:g=3.0,"
        "equalizer=f=900:width_type=q:width=1.2:g=4.0,"
        "equalizer=f=2200:width_type=q:width=1.0:g=5.0,"
        "lowpass=f=6500,"
        "agate=threshold=0.005:ratio=8:attack=3:release=90:range=0.03,"
        "acompressor=threshold=0.10:ratio=8:attack=1:release=55:makeup=2,"
        "alimiter=limit=0.90,"
        "loudnorm=I=-26:TP=-1.5:LRA=7"
    )
    ffmpeg_filter(flattened, destination, filters)


def make_monster(source: Path, destination: Path, temp: Path) -> None:
    """Monster voice (Pitch down -12 semitones, flattened F0 to D#2, low-end EQ boost)."""
    pitch_shifted = temp / "monster_pitch_shifted.wav"
    flattened = temp / "monster_flattened.wav"
    source_duration = duration_seconds(source)

    pitch_shift(source, pitch_shifted, -12.0, preserve_formants=True)
    flatten_f0(pitch_shifted, flattened, MONSTER_FLATTEN_HZ)

    filters = (
        "highpass=f=32,"
        "equalizer=f=82:width_type=q:width=1.0:g=7.0,"
        "equalizer=f=175:width_type=q:width=0.9:g=3.5,"
        "equalizer=f=1200:width_type=q:width=1.0:g=2.0,"
        "lowpass=f=7200,"
        "acompressor=threshold=0.14:ratio=5:attack=8:release=180:makeup=2,"
        "asoftclip=type=tanh:threshold=0.60:param=0.60:oversample=8,"
        "acrusher=bits=14:mix=0.12:aa=0.9,"
        "alimiter=limit=0.90,"
        "loudnorm=I=-16:TP=-1.5:LRA=7,"
        f"apad,atrim=end={source_duration:.6f}"
    )
    ffmpeg_filter(flattened, destination, filters)


def make_female_voice(source: Path, destination: Path, temp: Path) -> None:
    """To Female voice (+7 semitones pitch/formants shift, high-freq emphasis)."""
    pitch_shifted = temp / "to_female_pitched.wav"
    source_duration = duration_seconds(source)

    pitch_shift(source, pitch_shifted, 7.0, preserve_formants=False)
    filters = (
        "highpass=f=120,"
        "equalizer=f=250:width_type=q:width=1.0:g=-3.0,"
        "equalizer=f=2800:width_type=q:width=1.2:g=4.0,"
        "equalizer=f=5500:width_type=q:width=1.0:g=2.5,"
        "lowpass=f=11000,"
        "acompressor=threshold=0.12:ratio=4:attack=5:release=80:makeup=2,"
        "asoftclip=type=tanh:threshold=0.80:param=0.20:oversample=4,"
        "alimiter=limit=0.92,"
        "loudnorm=I=-16:TP=-1.5:LRA=7,"
        f"apad,atrim=end={source_duration:.6f}"
    )
    ffmpeg_filter(pitch_shifted, destination, filters)


def make_male_voice(source: Path, destination: Path, temp: Path) -> None:
    """To Male voice (-7 semitones pitch/formants shift, chest resonance boost)."""
    pitch_shifted = temp / "to_male_pitched.wav"
    source_duration = duration_seconds(source)

    pitch_shift(source, pitch_shifted, -7.0, preserve_formants=False)
    filters = (
        "highpass=f=55,"
        "equalizer=f=130:width_type=q:width=0.9:g=5.5,"
        "equalizer=f=300:width_type=q:width=1.0:g=2.0,"
        "equalizer=f=3500:width_type=q:width=1.5:g=-4.0,"
        "lowpass=f=7500,"
        "acompressor=threshold=0.15:ratio=5:attack=10:release=120:makeup=2,"
        "asoftclip=type=tanh:threshold=0.75:param=0.30:oversample=4,"
        "alimiter=limit=0.92,"
        "loudnorm=I=-16:TP=-1.5:LRA=7,"
        f"apad,atrim=end={source_duration:.6f}"
    )
    ffmpeg_filter(pitch_shifted, destination, filters)


# --- Registry and Dispatch ---

EFFECTS = {
    "donald": (make_donald, False),
    "robot": (make_robot, True),
    "monster": (make_monster, True),
    "female": (make_female_voice, False),
    "male": (make_male_voice, False),
}


def process_one(source: Path, output_dir: Path, selected_effects: list[str]) -> None:
    if not source.is_file():
        raise FileNotFoundError(f"Input file not found: {source}")

    stem = source.stem
    output_dir.mkdir(parents=True, exist_ok=True)
    orig_duration = duration_seconds(source)

    print(f"\nProcessing: {source.name} ({orig_duration:.3f}s)")

    with tempfile.TemporaryDirectory(prefix="voice_fx_") as temp_name:
        temp = Path(temp_name)
        for name in selected_effects:
            func, _ = EFFECTS[name]
            out_file = output_dir / f"{stem}_{name}.wav"
            print(f" -> Rendering effect: [{name}]...")
            func(source, out_file, temp)
            if out_file.exists():
                print(f"    Output: {out_file.name} ({duration_seconds(out_file):.3f}s)")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "inputs",
        nargs="*",
        type=Path,
        default=list(DEFAULT_INPUTS),
        help="Input mono WAV file(s); defaults to raw.wav.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=OUTPUT_DIR,
        help="Directory for rendered WAV files.",
    )
    parser.add_argument(
        "--effects",
        nargs="+",
        choices=list(EFFECTS.keys()),
        default=list(EFFECTS.keys()),
        help="Select specific effects to generate (default: all).",
    )

    args = parser.parse_args()

    # Determine if praat is actually needed based on chosen effects
    needs_praat = any(EFFECTS[fx][1] for fx in args.effects)

    try:
        require_tools(check_praat=needs_praat)
        for source in args.inputs:
            process_one(source.resolve(), args.output_dir.resolve(), args.effects)
        print("\nAll tasks completed successfully!")
    except (FileNotFoundError, RuntimeError, subprocess.CalledProcessError) as exc:
        print(f"\nError: {exc}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())