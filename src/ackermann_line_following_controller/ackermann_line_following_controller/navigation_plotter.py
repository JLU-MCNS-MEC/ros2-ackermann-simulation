"""Open reliable navigation plots using the upstream rqt_plot widgets."""

from __future__ import annotations

import sys
import signal

from python_qt_binding.QtCore import QTimer, Qt
from python_qt_binding.QtWidgets import QApplication
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rqt_plot.data_plot import DataPlot
from rqt_plot.plot_widget import PlotWidget


def plot_groups() -> tuple[tuple[str, tuple[str, ...]], ...]:
    """Return the fixed telemetry groups shown in separate plot windows."""
    return (
        (
            'Ackermann speed: command / smooth / applied / measured',
            (
                '/nav_diagnostics/speed/commanded/data',
                '/nav_diagnostics/speed/smoothed/data',
                '/nav_diagnostics/speed/applied/data',
                '/nav_diagnostics/speed/measured/data',
            ),
        ),
        (
            'Navigation decisions: path error / front lidar / action',
            (
                '/nav_diagnostics/navigation/cross_track_error/data',
                '/nav_diagnostics/obstacle/front_clearance/data',
                '/nav_diagnostics/decision/action_code/data',
            ),
        ),
    )


class NavigationPlotter:
    """Embed rqt_plot and retry topic attachment after ROS discovery."""

    def __init__(self, node: Node) -> None:
        """Create two non-overlapping upstream plot widgets."""
        self.node = node
        self.windows: list[PlotWidget] = []
        self.pending: list[tuple[PlotWidget, set[str]]] = []
        for index, (title, topics) in enumerate(plot_groups()):
            widget = PlotWidget(node)
            plot = DataPlot(widget)
            widget.switch_data_plot_widget(plot)
            plot.set_autoscale(x=False)
            plot.set_autoscale(
                y=DataPlot.SCALE_EXTEND | DataPlot.SCALE_VISIBLE
            )
            plot.set_xlim([0.0, 90.0])
            widget.setWindowTitle(title)
            widget.setGeometry(40 + index * 800, 80, 760, 430)
            widget.setWindowFlag(Qt.WindowStaysOnTopHint, True)
            widget.show()
            self.windows.append(widget)
            self.pending.append((widget, set(topics)))

        self.spin_timer = QTimer()
        self.spin_timer.timeout.connect(self._spin_once)
        self.spin_timer.start(10)
        self.discovery_timer = QTimer()
        self.discovery_timer.timeout.connect(self._attach_topics)
        self.discovery_timer.start(500)
        self._attach_topics()

    def _spin_once(self) -> None:
        if not rclpy.ok():
            self.spin_timer.stop()
            QApplication.quit()
            return
        try:
            rclpy.spin_once(self.node, timeout_sec=0.0)
        except (KeyboardInterrupt, RuntimeError) as error:
            if rclpy.ok() and not isinstance(error, KeyboardInterrupt):
                raise
            self.spin_timer.stop()
            QApplication.quit()

    def _attach_topics(self) -> None:
        for widget, topics in self.pending:
            for topic in tuple(topics):
                widget.add_topic(topic)
                if topic in widget._rosdata:
                    topics.remove(topic)
        if not any(topics for _, topics in self.pending):
            self.discovery_timer.stop()
            self.node.get_logger().info('All navigation plot topics attached')


def main(args=None) -> None:
    """Start the Qt event loop and rqt_plot telemetry windows."""
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('navigation_plotter')
    application = QApplication.instance() or QApplication(sys.argv[:1])
    previous_sigint = signal.signal(
        signal.SIGINT, lambda signum, frame: application.quit()
    )
    previous_sigterm = signal.signal(
        signal.SIGTERM, lambda signum, frame: application.quit()
    )
    plotter = NavigationPlotter(node)
    try:
        application.exec()
    finally:
        plotter.spin_timer.stop()
        plotter.discovery_timer.stop()
        try:
            node.destroy_node()
        except RuntimeError:
            pass
        if rclpy.ok():
            rclpy.shutdown()
        signal.signal(signal.SIGINT, previous_sigint)
        signal.signal(signal.SIGTERM, previous_sigterm)


if __name__ == '__main__':
    main()
