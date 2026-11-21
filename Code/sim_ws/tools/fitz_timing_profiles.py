"""Matched synthetic timing profiles for the Fitzpatrick controller experiment.

Each row describes one 50 ms control release in units of 10 ms of plant time.
The two age fields refer to the latest PF pose and odometry visible at release.
The last field is the entire release-to-command-arrival delay. Solver wall time
is measured separately and must not be added to this field.

The measured v8 trajectory archive contains received pose/time, without a
per-command link to its input pose and final driver event. The archived laptop
and Jetson interface replays are synthetic. These profiles are designed
stress conditions, not distributions fitted to physical laps.
"""

from random import Random
from typing import NamedTuple


class TimingSample(NamedTuple):
    pose_age_steps: int
    odom_age_steps: int
    command_delay_steps: int


PROFILES = ("sync", "mean", "shuffled", "ordered")
WINDOW_TICKS = 40  # shuffle within a two-second release-time window
ZERO = TimingSample(0, 0, 0)
NORMAL = TimingSample(2, 1, 1)
LATE = TimingSample(6, 5, 9)
# Three normal rows per late row. Differences in pose and odometry ages are
# less than one control release, so source stamps never move backwards.
MEAN = TimingSample(3, 2, 3)


def make_schedule(profile, seed, ticks):
    """Generate an immutable schedule, shared across methods for a seed.

    Every four releases contribute one late sample. Complete windows have ten
    consecutive late rows at a seeded position in ordered mode. Shuffling the
    *rows* preserves cross-channel dependence and the histogram in each time
    window. This is temporal blocking, not stratification by road geometry.

    The constant profile is the exact mean for horizons divisible by four.
    Other horizons truncate the final partial window to an integer number of
    events. Early termination can likewise truncate the observed realized mix.
    """
    if profile not in PROFILES:
        raise ValueError(f"unknown timing profile {profile!r}")
    if not isinstance(ticks, int) or ticks <= 0:
        raise ValueError("ticks must be a positive integer")
    if profile in ("sync", "mean"):
        return (ZERO if profile == "sync" else MEAN,) * ticks

    placement = Random(seed)
    # A separate stream makes burst positions independent of the permutation.
    permutation = Random(f"fitz-timing-shuffle/{seed}")
    schedule = []
    for offset in range(0, ticks, WINDOW_TICKS):
        size = min(WINDOW_TICKS, ticks - offset)
        late_count = size // 4
        first_late = placement.randrange(size - late_count + 1)
        window = [NORMAL] * size
        window[first_late:first_late + late_count] = [LATE] * late_count
        if profile == "shuffled":
            permutation.shuffle(window)
        schedule.extend(window)
    return tuple(schedule)
