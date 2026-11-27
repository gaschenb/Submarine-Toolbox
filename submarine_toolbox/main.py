"""Entry point: `submarine-toolbox` or `python -m submarine_toolbox.main`."""

import sys

def _theme(app):
    from PyQt5 import QtGui
    import pyqtgraph as pg
    app.setStyle("Fusion")
    p = QtGui.QPalette()
    c = QtGui.QColor
    for role, col in [
            (QtGui.QPalette.Window, c(22, 28, 38)),
            (QtGui.QPalette.WindowText, c(210, 222, 235)),
            (QtGui.QPalette.Base, c(14, 19, 27)),
            (QtGui.QPalette.AlternateBase, c(26, 33, 45)),
            (QtGui.QPalette.Text, c(210, 222, 235)),
            (QtGui.QPalette.Button, c(34, 42, 56)),
            (QtGui.QPalette.ButtonText, c(215, 225, 238)),
            (QtGui.QPalette.Highlight, c(40, 120, 180)),
            (QtGui.QPalette.HighlightedText, c(255, 255, 255)),
            (QtGui.QPalette.ToolTipBase, c(30, 38, 50)),
            (QtGui.QPalette.ToolTipText, c(220, 230, 240))]:
        p.setColor(role, col)
    p.setColor(QtGui.QPalette.Disabled, QtGui.QPalette.Text, c(110, 120, 135))
    p.setColor(QtGui.QPalette.Disabled, QtGui.QPalette.ButtonText, c(110, 120, 135))
    app.setPalette(p)
    app.setStyleSheet(
        "QDockWidget::title{background:#1b2331;padding:4px 6px;}"
        "QTabBar::tab{background:#1b2331;color:#9fb2c8;padding:5px 10px;"
        "border:1px solid #273246;border-bottom:none;}"
        "QTabBar::tab:selected{background:#263247;color:#e6eef8;}"
        "QProgressBar{border:1px solid #2d3a50;border-radius:3px;background:#0f151e;"
        "text-align:center;color:#dfe8f2;}"
        "QProgressBar::chunk{background:#2f7fb5;border-radius:2px;}"
        "QToolBar{background:#161d29;border:none;spacing:4px;}"
        "QStatusBar{color:#8094ab;}")
    pg.setConfigOptions(antialias=True, background="#0b111a", foreground="#a9bccf")

def main():
    from PyQt5 import QtCore, QtWidgets
    QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling, True)
    QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_UseHighDpiPixmaps, True)
    app = QtWidgets.QApplication(sys.argv)
    _theme(app)
    from submarine_toolbox.gui.main_window import MainWindow
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
