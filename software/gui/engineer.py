from PyQt5.QtCore import (QCoreApplication, QMetaObject, QRect)
from PyQt5.QtGui import (QIcon, QPixmap, QFont, QFontDatabase)
from PyQt5.QtWidgets import (QLabel, QPushButton)
import os
import subprocess
from stylesheet import Engineer_buttons_st, red_button, back_st
from utils import BG_path, scale  # Make sure scale is imported

class EngineerUi(object):
    def setupUi(self, Dialog):
        Dialog.setObjectName("Dialog")
        Dialog.resize(scale(928), scale(596))  # Scaled window size
        
        # Loading font
        script_dir = os.path.dirname(os.path.abspath(__file__))
        font_path = os.path.join(script_dir, "GillSans.ttf")
        id = QFontDatabase.addApplicationFont(font_path)
        if id == -1:
            print("Failed to load font!")
        
        families = QFontDatabase.applicationFontFamilies(id)
        # Scalable font sizes
        font = QFont(families[0], 18)
        Afont = QFont(families[0], 14)
        ICCfont = QFont(families[0], 17)

        # Background
        self.Bg_label = QLabel(Dialog)
        self.Bg_label.setObjectName("Background label")
        self.Bg_label.setGeometry(QRect(scale(-3), scale(-5), scale(945), scale(607)))
        self.Bg_label.setPixmap(QPixmap(BG_path))
        self.Bg_label.setScaledContents(True)

        # Back button
        self.BackButton = QPushButton(Dialog)
        self.BackButton.setObjectName("Back Button")
        self.BackButton.setGeometry(QRect(scale(10), scale(10), scale(61), scale(41)))
        icon = QIcon.fromTheme("go-previous")
        self.BackButton.setIcon(icon)
        self.BackButton.setStyleSheet(back_st)
        self.BackButton.setFont(Afont)

        # Depth Estimation
        self.DepthButton = QPushButton(Dialog)
        self.DepthButton.setObjectName("Depth Estimation Button")
        self.DepthButton.setGeometry(QRect(scale(290), scale(150), scale(351), scale(81)))
        self.DepthButton.setStyleSheet(Engineer_buttons_st)
        self.DepthButton.setFont(font)

        # Invasive Carp Computer Model
        self.IccButton = QPushButton(Dialog)
        self.IccButton.setObjectName("Invasive Carp Computer Model Button")
        self.IccButton.setGeometry(QRect(scale(290), scale(240), scale(351), scale(81)))
        self.IccButton.setStyleSheet(Engineer_buttons_st)
        self.IccButton.setFont(ICCfont)
        self.IccButton.clicked.connect(self.openICC)

        # Recording Button
        self.RecButton = QPushButton(Dialog)
        self.RecButton.setObjectName("Start Recording for Photosphere Task")
        self.RecButton.setGeometry(QRect(scale(290), scale(330), scale(351), scale(81)))
        self.RecButton.setStyleSheet(Engineer_buttons_st)
        self.RecButton.setFont(font)

        # Stop recording Button
        self.StopButton = QPushButton(Dialog)
        self.StopButton.setObjectName("Stop Recording for Photosphere Task")
        self.StopButton.setGeometry(QRect(scale(370), scale(420), scale(181), scale(51)))
        self.StopButton.setStyleSheet(red_button)
        self.StopButton.setFont(Afont)

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

    def openICC(self):
        subprocess.Popen(['python3', '/home/abdelrhman/MATE-ROV-2025/software/invasive-carp-model/main.py'])
