"""整手触觉热力图页面。"""

from __future__ import annotations

import numpy as np

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import QLabel, QHBoxLayout, QTabWidget, QVBoxLayout, QWidget

from app.platform_compat import configure_matplotlib_fonts

configure_matplotlib_fonts()

from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

from app.models.hand_state import HandState
from app.models.tactile_layout import (
    empty_tactile_patches_for_profile,
    normalized_display_patches,
    patch_groups_for_profile,
    patch_labels_for_profile,
    patch_shapes_for_profile,
)


def _patch_title(patch_name: str) -> str:
    return patch_name


class PatchGroupCanvas(FigureCanvas):
    def __init__(self, patch_names: tuple[str, ...], shapes: dict[str, tuple[int, int]], labels: dict[str, str], parent=None):
        self.patch_names = patch_names
        self.shapes = shapes
        self.labels = labels
        self.figure = Figure(figsize=(8, 5), facecolor="white", constrained_layout=True)
        super().__init__(self.figure)
        self.setParent(parent)

        axes = self.figure.subplots(1, len(patch_names), squeeze=False)[0]
        self.axes = list(axes)
        self.images = []

        for ax, patch_name in zip(self.axes, patch_names):
            rows, cols = self.shapes[patch_name]
            data = np.zeros((rows, cols), dtype=np.float32)
            image = ax.imshow(
                data,
                cmap="turbo",
                interpolation="nearest",
                vmin=0.0,
                vmax=1.0,
                aspect="equal",
            )
            ax.set_title(self.labels.get(patch_name, _patch_title(patch_name)), fontsize=12)
            ax.set_xticks(np.arange(cols))
            ax.set_yticks(np.arange(rows))
            ax.tick_params(labelsize=7, length=0)
            ax.grid(color="#222222", linewidth=0.35)
            ax.set_xticks(np.arange(-0.5, cols, 1), minor=True)
            ax.set_yticks(np.arange(-0.5, rows, 1), minor=True)
            ax.grid(which="minor", color="#222222", linewidth=0.35)
            self.images.append(image)

        self.figure.colorbar(self.images[-1], ax=self.axes, fraction=0.025, pad=0.02)
        self.draw()

    def update_patches(self, patches: dict[str, np.ndarray]):
        for image, patch_name in zip(self.images, self.patch_names):
            data = np.asarray(
                patches.get(patch_name, np.zeros(self.shapes[patch_name])),
                dtype=np.float32,
            )
            if data.shape != self.shapes[patch_name]:
                data = np.zeros(self.shapes[patch_name], dtype=np.float32)
            image.set_data(np.clip(data, 0.0, 1.0))
        self.draw_idle()


class TactilePage(QWidget):
    def __init__(self, hand_state: HandState, parent=None):
        super().__init__(parent)
        self.hand_state = hand_state
        self._canvases: list[PatchGroupCanvas] = []
        self._current_patch_names: tuple[str, ...] = ()
        self._rebuilding_tabs = False

        layout = QVBoxLayout(self)
        layout.setContentsMargins(24, 24, 24, 24)
        layout.setSpacing(12)

        top_bar = QHBoxLayout()
        title = QLabel("整手触觉热力图")
        title.setStyleSheet("font-size: 18px; font-weight: bold; color: #333;")
        top_bar.addWidget(title)
        top_bar.addStretch()
        layout.addLayout(top_bar)

        self.tabs = QTabWidget()
        self.tabs.setDocumentMode(True)
        self.tabs.setStyleSheet(
            "QTabWidget::pane { border: 1px solid #ddd; border-radius: 4px; }"
            "QTabBar::tab { padding: 8px 16px; font-size: 13px; }"
            "QTabBar::tab:selected { color: #0078d4; font-weight: bold; }"
        )

        self.tabs.currentChanged.connect(lambda _index: self._update_plot())

        layout.addWidget(self.tabs, 1)

        info_bar = QHBoxLayout()
        self.info_max = QLabel("最大值: 0.00")
        self.info_mean = QLabel("平均值: 0.00")
        self.info_active = QLabel("激活点: 0")
        for label in (self.info_max, self.info_mean, self.info_active):
            label.setStyleSheet(
                "font-size: 13px; padding: 4px 8px; background: #f0f0f0; border-radius: 4px;"
            )
            info_bar.addWidget(label)
        info_bar.addStretch()
        layout.addLayout(info_bar)

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._update_plot)
        self.hand_state.profile_changed.connect(lambda _profile: self._rebuild_tabs())
        self._rebuild_tabs()
        self._update_plot()

    def _rebuild_tabs(self):
        if self._rebuilding_tabs:
            return
        self._rebuilding_tabs = True
        signals_were_blocked = self.tabs.blockSignals(True)
        if getattr(self.hand_state.profile, "tactile_layout", ()):
            shapes = patch_shapes_for_profile(self.hand_state.profile)
            labels = patch_labels_for_profile(self.hand_state.profile)
            groups = patch_groups_for_profile(self.hand_state.profile)
        else:
            patches = self.hand_state.tactile_patches
            if patches:
                shapes = {
                    name: tuple(np.asarray(patch).shape[:2])
                    for name, patch in patches.items()
                    if np.asarray(patch).ndim == 2
                }
                labels = {name: _patch_title(name) for name in shapes}
                groups = [("触觉", tuple(shapes.keys()))]
            else:
                shapes = patch_shapes_for_profile(self.hand_state.profile)
                labels = patch_labels_for_profile(self.hand_state.profile)
                groups = patch_groups_for_profile(self.hand_state.profile)
        self._current_patch_names = tuple(shapes.keys())
        try:
            self.tabs.clear()
            self._canvases.clear()
            if not groups:
                groups = [("触觉", tuple(shapes.keys()))]
            for tab_name, patch_names in groups:
                existing = tuple(name for name in patch_names if name in shapes)
                if not existing:
                    continue
                tab = QWidget()
                tab_layout = QVBoxLayout(tab)
                tab_layout.setContentsMargins(12, 12, 12, 12)
                canvas = PatchGroupCanvas(existing, shapes, labels, tab)
                tab_layout.addWidget(canvas, 1)
                self.tabs.addTab(tab, tab_name)
                self._canvases.append(canvas)
        finally:
            self.tabs.blockSignals(signals_were_blocked)
            self._rebuilding_tabs = False

    def _update_plot(self):
        if self._rebuilding_tabs:
            return
        patches = self.hand_state.tactile_patches
        if not patches:
            patches = empty_tactile_patches_for_profile(self.hand_state.profile, dtype=np.float32)
        patches = normalized_display_patches(patches, self.hand_state.profile)
        patch_names = tuple(patches.keys())
        if patch_names and patch_names != self._current_patch_names:
            self._rebuild_tabs()

        current = self.tabs.currentIndex()
        if 0 <= current < len(self._canvases):
            self._canvases[current].update_patches(patches)

        arrays = [np.asarray(patch, dtype=np.float32).reshape(-1) for patch in patches.values()]
        values = np.concatenate(arrays) if arrays else np.zeros(1, dtype=np.float32)
        self.info_max.setText(f"最大值: {float(np.max(values)):.2f}")
        self.info_mean.setText(f"平均值: {float(np.mean(values)):.2f}")
        self.info_active.setText(f"激活点: {int(np.sum(values > 0.2))}")

    def cleanup(self):
        self._timer.stop()

    def showEvent(self, event):
        super().showEvent(event)
        self._update_plot()
        self._timer.start(200)

    def hideEvent(self, event):
        super().hideEvent(event)
        self._timer.stop()
