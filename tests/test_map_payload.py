from pathlib import Path
import unittest

from hpasim.map_payload import build_map_payload
from hpasim.planner import load_opendrive_map


MAP_PATH = Path("maps/opendrive/parking_lot_full.xodr")


class MapPayloadTest(unittest.TestCase):
    def test_exports_interactive_viewer_payload(self) -> None:
        opendrive_map = load_opendrive_map(MAP_PATH)

        payload = build_map_payload(opendrive_map)

        self.assertEqual(payload["scenario"]["key"], "parking_lot_full")
        self.assertIn("egoStart", payload["scenario"])
        self.assertGreater(len(payload["roads"]), 0)
        parking_spaces = [obj for obj in payload["objects"] if obj["type"] == "parkingSpace"]
        self.assertGreater(len(parking_spaces), 20)
        self.assertTrue(all(space["polygon"] for space in parking_spaces))


if __name__ == "__main__":
    unittest.main()
