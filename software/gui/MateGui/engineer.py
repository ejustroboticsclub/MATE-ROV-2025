
from PyQt5.QtCore import (QCoreApplication,QMetaObject, QRect)
from PyQt5.QtGui import (QIcon, QPixmap)
from PyQt5.QtWidgets import (QLabel, QPushButton)

from stylesheet import Engineer_buttons_st , red_button


class EngineerUi(object):
    def setupUi(self, Dialog):
        Dialog.setObjectName("Dialog")
        Dialog.resize(928, 596)
        
        # Background
        self.Bg_label = QLabel(Dialog)
        self.Bg_label.setObjectName("Background label")
        self.Bg_label.setGeometry(QRect(-3, -5, 931, 601))
        self.Bg_label.setPixmap(QPixmap("Visuals/Background(final).jpg"))
        self.Bg_label.setScaledContents(True)

        # Back button
        self.BackButton = QPushButton(Dialog)
        self.BackButton.setObjectName("Back Button")
        self.BackButton.setGeometry(QRect(10, 10, 61, 41))
        icon = QIcon.fromTheme("go-previous")  
        self.BackButton.setIcon(icon)

        # Depth Estimation
        self.DepthButton = QPushButton(Dialog)
        self.DepthButton.setObjectName("Depth Estimation Button")
        self.DepthButton.setGeometry(QRect(290, 150, 351, 81))
        self.DepthButton.setStyleSheet(Engineer_buttons_st)

        # Invasive Carp Computer Model
        self.IccButton = QPushButton(Dialog)
        self.IccButton.setObjectName("Invasive Carp Computer Model Button")
        self.IccButton.setGeometry(QRect(290, 240, 351, 81))
        self.IccButton.setStyleSheet(Engineer_buttons_st)

        # Recording Button
        self.RecButton = QPushButton(Dialog)
        self.RecButton.setObjectName("Start Recording for Photosphere Task")
        self.RecButton.setGeometry(QRect(290, 330, 351, 81))
        self.RecButton.setStyleSheet(Engineer_buttons_st)

        # Stop recording Button
        self.StopButton = QPushButton(Dialog)
        self.StopButton.setObjectName("Stop Recording for Photosphere Task")
        self.StopButton.setGeometry(QRect(370, 420, 181, 51))
        self.StopButton.setStyleSheet(red_button)
        self.setText(Dialog)

        QMetaObject.connectSlotsByName(Dialog)

    def setText(self, Dialog):
        Dialog.setWindowTitle(QCoreApplication.translate("Dialog", "Dialog", None))
        self.Bg_label.setText("")
        self.BackButton.setText(QCoreApplication.translate("Dialog", "Back", None))
        self.DepthButton.setText(QCoreApplication.translate("Dialog", "Depth Estimation", None))
        self.IccButton.setText(QCoreApplication.translate("Dialog", "Invasive Carp Computer Model", None))
        self.RecButton.setText(QCoreApplication.translate("Dialog", "Start Recording (Photosphere)", None))
        self.StopButton.setText(QCoreApplication.translate("Dialog", "Stop Recording", None))


   