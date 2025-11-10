"""Custom Qt widgets used across the SmartHand UI."""

from PyQt5.QtCore import QPoint, Qt, pyqtSignal
from PyQt5.QtWidgets import QLabel, QScrollArea


class ClickableLabel(QLabel):
    """QLabel that emits a signal when clicked."""

    clicked = pyqtSignal(QPoint)

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def mousePressEvent(self, event):  # noqa: D401
        if event.button() == Qt.LeftButton:
            self.clicked.emit(event.pos())
        super().mousePressEvent(event)


class ZoomScrollArea(QScrollArea):
    """Scroll area that emits wheel events for zooming."""

    wheel_zoom = pyqtSignal(QPoint, int)

    def wheelEvent(self, event):  # noqa: D401
        delta = event.angleDelta().y()
        if not delta:
            super().wheelEvent(event)
            return

        content = self.widget()
        if content is not None:
            content_pos = content.mapFrom(self.viewport(), event.pos())
        else:
            content_pos = event.pos()

        self.wheel_zoom.emit(content_pos, delta)
        event.accept()


__all__ = ["ClickableLabel", "ZoomScrollArea"]
