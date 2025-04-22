from PyQt5.QtCore import (QCoreApplication, QRect, QSize,QMetaObject)
from PyQt5.QtGui import (QIcon, QPixmap)
from PyQt5.QtWidgets import (QLabel, QPushButton)

from stylesheet import Laning_buttons_st
from utils import BG_path ,copilot_path,pilot_path,engineer_path,float_path , scale


class LandingPageUi(object):
    def setupUi(self, Dialog):
        Dialog.setObjectName("Dialog")
        Dialog.resize(scale(930), scale(599))
        
        # Background
        self.Bg_label = QLabel(Dialog)
        self.Bg_label.setObjectName("BG label")
        self.Bg_label.setGeometry(QRect(scale(-3), scale(-5), scale(945), scale(607)))
        self.Bg_label.setPixmap(QPixmap(BG_path))
        self.Bg_label.setScaledContents(True)

        # Pilot button
        self.PilotButton = QPushButton(Dialog)
        self.PilotButton.setObjectName("Pilot Button")
        self.PilotButton.setGeometry(QRect(scale(240), scale(100), scale(201), scale(201)))
        self.PilotButton.setStyleSheet(Laning_buttons_st)
        icon = QIcon()
        icon.addFile(pilot_path, QSize(), QIcon.Normal, QIcon.Off)
        self.PilotButton.setIcon(icon)
        self.PilotButton.setIconSize(QSize(scale(158), scale(140)))

        # Copilot button
        self.CoButton = QPushButton(Dialog)
        self.CoButton.setObjectName("Copilot Buttton")
        self.CoButton.setGeometry(QRect(scale(490), scale(100), scale(201), scale(201)))
        self.CoButton.setStyleSheet(Laning_buttons_st)
        icon1 = QIcon()
        icon1.addFile(copilot_path, QSize(), QIcon.Normal, QIcon.Off)
        self.CoButton.setIcon(icon1)
        self.CoButton.setIconSize(QSize(scale(158), scale(135)))

        # Engineer button
        self.EngButton = QPushButton(Dialog)
        self.EngButton.setObjectName("Enginner Button")
        self.EngButton.setGeometry(QRect(scale(240), scale(330), scale(201), scale(191)))
        self.EngButton.setStyleSheet(Laning_buttons_st)
        icon2 = QIcon()
        icon2.addFile(engineer_path, QSize(), QIcon.Normal, QIcon.Off)
        self.EngButton.setIcon(icon2)
        self.EngButton.setIconSize(QSize(scale(140), scale(140)))

        # Float button
        self.FloatButton = QPushButton(Dialog)
        self.FloatButton.setObjectName("Float Button")
        self.FloatButton.setGeometry(QRect(scale(489), scale(331), scale(201), scale(191)))
        self.FloatButton.setStyleSheet(Laning_buttons_st)
        icon3 = QIcon()
        icon3.addFile(float_path, QSize(), QIcon.Normal, QIcon.Off)
        self.FloatButton.setIcon(icon3)
        self.FloatButton.setIconSize(QSize(scale(160), scale(160)))

        self.setText(Dialog)

        QMetaObject.connectSlotsByName(Dialog)

    def setText(self, Dialog):
        Dialog.setWindowTitle(QCoreApplication.translate("Dialog", "Dialog", None))
        self.Bg_label.setText("")
        self.PilotButton.setText("")
        self.CoButton.setText("")
        self.EngButton.setText("")
        self.FloatButton.setText("")