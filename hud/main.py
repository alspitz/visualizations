# encoding: utf-8

import multiprocessing
import sys

from PyQt5.QtCore import QObject, QTimer, QUrl, pyqtSignal, pyqtSlot, Qt
from PyQt5.QtGui import QFont, QGuiApplication
from PyQt5.QtWidgets import (
    QApplication, QLabel, QWidget, QHBoxLayout, QVBoxLayout, QDesktopWidget
)
#from PyQt5.QtQml import QQmlComponent, QQmlEngine, QQmlApplicationEngine

from data import DataSource
from data_ser import DataSer

class Info:
  update_ms = 16

  font = QFont('SansSerif', 50)
  title_font = QFont('SansSerif', 60)

  window_sheet = 'background-color: #e0e0e0;'
  label_sheet = 'QLabel {color: #bb0000;}'

  title = "RISLAB Telemetry"

  def __init__(self):
    self.data_ser = DataSer()
    self.process = multiprocessing.Process(target=self.ros_process)
    self.process.start()

    self.title_font.setBold(True)

    self.labels = []

    self.w = QWidget()
    self.w.resize(1280, 720)
    self.w.setWindowTitle(self.title)
    self.w.setStyleSheet(self.window_sheet)

    #self.engine = QQmlApplicationEngine()
    #self.context = self.engine.rootContext()
    #self.engine.load('telem.qml')
    #self.comp = QQmlComponent(self.engine)
    #self.comp.loadUrl(QUrl('telem.qml'))

    #level = self.comp.create()
    #if level is not None:
    #  self.engine.show()

    #else:
    #  for err in self.comp.errors():
    #    print(err.toString())
    #    sys.exit(1)

    main_vbox = QVBoxLayout()
    self.w.setLayout(main_vbox)

    hbox1 = QHBoxLayout()
    main_vbox.addLayout(hbox1)

    cols = [
        ("Pos Err", "pos", 3),
        ("Vel Err", "vel", 3),
        ("Accel Dist", "accel_dist", 3),
        ("Torque Dist", "torque_dist", 3)
    ]

    self.updates = []

    for col_name, col_id, n in cols:
      vbox = QVBoxLayout()
      hbox1.addLayout(vbox)

      title_label = QLabel(col_name)
      title_label.setFont(self.title_font)
      title_label.setStyleSheet(self.label_sheet)
      vbox.addWidget(title_label)

      for i in range(n):
        label = QLabel("", self.w)
        label.setStyleSheet(self.label_sheet)
        label.setAlignment(Qt.AlignRight)
        self.updates.append((label, "", (col_id, i), "%0.3f"))
        label.setFont(self.font)
        vbox.addWidget(label)
        self.labels.append(label)

      hbox1.addSpacing(70)

    hbox3 = QHBoxLayout()
    main_vbox.addLayout(hbox3)

    title_label = QLabel("Yaw Err: ")
    title_label.setFont(self.title_font)
    title_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
    title_label.setStyleSheet(self.label_sheet)
    hbox3.addWidget(title_label)

    label = QLabel("", self.w)
    label.setStyleSheet(self.label_sheet)
    label.setAlignment(Qt.AlignVCenter)
    label.setFont(self.font)
    self.updates.append((label, "", ("yawerr", 0), "%0.3f°"))
    hbox3.addWidget(label)

    hbox2 = QHBoxLayout()
    main_vbox.addLayout(hbox2)

    cols = [
        ("Euler (RPY)", "euler", 3),
        ("RPM", "rpms", 4),
    ]

    for col_name, col_id, n in cols:
      vbox = QVBoxLayout()
      hbox2.addLayout(vbox)

      title_label = QLabel(col_name)
      title_label.setAlignment(Qt.AlignCenter)
      title_label.setFont(self.title_font)
      title_label.setStyleSheet(self.label_sheet)
      vbox.addWidget(title_label)

      fmt_s = "%0.3f°" if col_id == "euler" else "%05d"

      for i in range(n):
        label = QLabel("", self.w)
        label.setAlignment(Qt.AlignRight)
        label.setStyleSheet(self.label_sheet)
        self.updates.append((label, "", (col_id, i), fmt_s))
        label.setFont(self.font)
        vbox.addWidget(label)
        self.labels.append(label)

      hbox2.addSpacing(70)

    vbox_imu = QVBoxLayout()
    hbox2.addLayout(vbox_imu)

    vbox_imu.addSpacing(70)

    label = QLabel("")
    label.setAlignment(Qt.AlignRight)
    label.setFont(self.font)
    label.setStyleSheet(self.label_sheet)
    self.updates.append((label, "Voltage:", ("voltage", 0), "%2.2f V"))
    vbox_imu.addWidget(label)

    label = QLabel("")
    label.setAlignment(Qt.AlignRight)
    label.setFont(self.font)
    label.setStyleSheet(self.label_sheet)
    self.updates.append((label, "Current:", ("current", 0), "%2.2f A"))
    vbox_imu.addWidget(label)

    label = QLabel("")
    label.setAlignment(Qt.AlignRight)
    label.setFont(self.font)
    label.setStyleSheet(self.label_sheet)
    self.updates.append((label, "Temp:", ("temp", 0), "%2.2f °C"))
    vbox_imu.addWidget(label)

    hbox2.addSpacing(70)

    qr = self.w.frameGeometry()
    cp = QDesktopWidget().availableGeometry().center()
    qr.moveCenter(cp)
    self.w.move(qr.topLeft())

    self.w.show()

    self.update_timer = QTimer(self.w)
    self.update_timer.setSingleShot(False)
    self.update_timer.timeout.connect(self.update)
    self.update_timer.start(self.update_ms)

    self.time_ms = 0

  def ros_process(self):
    data = DataSource(self.data_ser)

  def update(self):
    data = self.data_ser.read()
    self.time_ms += self.update_ms
    for label, prefix, label_ids, fmt_s in self.updates:
      label.setText(("%s " + fmt_s) % (prefix, getattr(data, label_ids[0])[label_ids[1]]))

if __name__ == "__main__":
  app = QApplication(sys.argv)
  info = Info()
  app.exec_()
