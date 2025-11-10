"""Entry point for the SmartHand desktop application."""

import sys

from PyQt5.QtWidgets import QApplication

from smarthand.app import SmartHandApp


def main() -> None:
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    window = SmartHandApp()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
