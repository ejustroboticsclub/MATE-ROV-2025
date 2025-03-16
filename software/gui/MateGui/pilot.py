from PyQt5.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PyQt5.QtGui import (QBrush, QColor, QConicalGradient, QCursor,
    QFont, QFontDatabase, QGradient, QIcon,
    QImage, QKeySequence, QLinearGradient, QPainter,
    QPalette, QPixmap, QRadialGradient, QTransform)
from PyQt5.QtWidgets import (QApplication, QDialog, QLabel, QPushButton,
    QSizePolicy, QWidget)

from stylesheet import Engineer_buttons_st , back_st
import os
from utils import CameraStreamer , BG_path

class PilotUi(object):
    def setupUi(self, Dialog):
        font_path = os.path.abspath("GillSans.ttf")


        # Try loading the font
        id = QFontDatabase.addApplicationFont(font_path)
        if id == -1:
            print("Failed to load font!")
        
        families = QFontDatabase.applicationFontFamilies(id)
        font=QFont(families[0],22)
        Afont=QFont(families[0],11)

        Dialog.setObjectName("Dialog")
        Dialog.resize(928, 596)

        # Background
        self.Bg_label = QLabel(Dialog)
        self.Bg_label.setObjectName("Background")
        self.Bg_label.setGeometry(QRect(-3, -5, 945, 607))
        self.Bg_label.setPixmap(QPixmap(BG_path))
        self.Bg_label.setScaledContents(True)

        # Back button
        self.BackButton = QPushButton(Dialog)
        self.BackButton.setObjectName("Back button")
        self.BackButton.setGeometry(QRect(10, 10, 61, 41))
        icon = QIcon()
        icon = QIcon.fromTheme("go-previous")  
        self.BackButton.setIcon(icon)
        self.BackButton.setStyleSheet(back_st)
        self.BackButton.setFont(Afont)

        # Camera system button
        self.CamButton = QPushButton(Dialog)
        self.CamButton.setObjectName("Launching the Camera system button")
        self.CamButton.setGeometry(QRect(290, 240, 351, 81))
        self.CamButton.setStyleSheet(Engineer_buttons_st)
        self.CamButton.setFont(font)

        #IPs passed for cameraStreamer class
        IPS = [
            "rtsp://192.168.216.56:8554/camerafeed1",
            "rtsp://192.168.216.56:8554/camerafeed1",
            "rtsp://192.168.216.56:8554/camerafeed1",
            "rtsp://192.168.216.56:8554/camerafeed2",
            "rtsp://192.168.216.56:8554/camerafeed2",
            "rtsp://192.168.216.56:8554/camerafeed2"
        ]
        self.camera_6feeds = CameraStreamer(IPS)
        self.CamButton.clicked.connect(self.camera_6feeds.run)

        self.setText(Dialog)

        QMetaObject.connectSlotsByName(Dialog)
    

    def setText(self, Dialog):
        Dialog.setWindowTitle(QCoreApplication.translate("Dialog", "Dialog", None))
        self.Bg_label.setText("")
        self.BackButton.setText(QCoreApplication.translate("Dialog", "Back", None))
        self.CamButton.setText(QCoreApplication.translate("Dialog", "Launch Camera System", None))