import json
import unittest

from scripts.serve_viewer import ViewerHandler, ViewerState


class DummyHandler:
    state = ViewerState()


class ViewerApiTest(unittest.TestCase):
    def test_drive_endpoint_payload_contains_trajectory_states(self) -> None:
        handler = DummyHandler()
        payloads = []

        def write_json(status, payload):
            payloads.append((status, payload))

        handler._read_json = lambda: {"target": "central_upper_angled_row_13"}  # type: ignore[attr-defined]
        handler._write_json = write_json  # type: ignore[attr-defined]

        ViewerHandler._handle_drive(handler)  # type: ignore[arg-type]

        status, payload = payloads[0]
        self.assertEqual(status.value, 200)
        self.assertEqual(payload["target"], "central_upper_angled_row_13")
        self.assertGreater(len(payload["points"]), 20)
        first = payload["points"][0]
        self.assertIn("yaw", first)
        self.assertIn("gear", first)
        self.assertIn("steer", first)
        self.assertIn("acceleration", first)
        self.assertIn("curvature", first)
        json.dumps(payload)


if __name__ == "__main__":
    unittest.main()
