from pathlib import Path
import unittest


VIEWER_DIR = Path("viewer")


class ViewerAssetsTest(unittest.TestCase):
    def test_viewer_uses_external_css_and_javascript(self) -> None:
        html = (VIEWER_DIR / "index.html").read_text(encoding="utf-8")

        self.assertIn('href="styles.css"', html)
        self.assertIn('src="app.js"', html)
        self.assertNotIn("<style>", html)
        self.assertNotIn("const canvas =", html)
        self.assertTrue((VIEWER_DIR / "styles.css").is_file())
        self.assertTrue((VIEWER_DIR / "app.js").is_file())


if __name__ == "__main__":
    unittest.main()
