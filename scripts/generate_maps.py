"""Generate HPASim OpenDRIVE parking scenarios."""

from pathlib import Path
import sys


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from hpasim.parking_scenarios import write_all_scenarios


def main() -> None:
    for path in write_all_scenarios():
        print(path)


if __name__ == "__main__":
    main()
