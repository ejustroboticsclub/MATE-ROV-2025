from PyQt5.QtCore import (QCoreApplication, QRect, QSize,QMetaObject)
from PyQt5.QtGui import (QIcon, QPixmap)
from PyQt5.QtWidgets import (QLabel, QPushButton)

from stylesheet import Laning_buttons_st
class LandingPageUi(object):
    def setupUi(self, Dialog):
        Dialog.setObjectName("Dialog")
        Dialog.resize(930, 599)
        
        # Background
        self.Bg_label = QLabel(Dialog)
        self.Bg_label.setObjectName("BG label")
        self.Bg_label.setGeometry(QRect(-3, -5, 931, 601))
        self.Bg_label.setPixmap(QPixmap("Visuals/Background(final).jpg"))
        self.Bg_label.setScaledContents(True)

        # Pilot button
        self.PilotButton = QPushButton(Dialog)
        self.PilotButton.setObjectName("Pilot Button")
        self.PilotButton.setGeometry(QRect(240, 100, 201, 201))
        self.PilotButton.setStyleSheet(Laning_buttons_st)
        icon = QIcon()
        icon.addFile("Visuals/pilot(final).png", QSize(), QIcon.Normal, QIcon.Off)
        self.PilotButton.setIcon(icon)
        self.PilotButton.setIconSize(QSize(158, 140))

        # Copilot button
        self.CoButton = QPushButton(Dialog)
        self.CoButton.setObjectName("Copilot Buttton")
        self.CoButton.setGeometry(QRect(490, 100, 201, 201))
        self.CoButton.setStyleSheet(Laning_buttons_st)
        icon1 = QIcon()
        icon1.addFile("Visuals/copilot(final).png", QSize(), QIcon.Normal, QIcon.Off)
        self.CoButton.setIcon(icon1)
        self.CoButton.setIconSize(QSize(158, 135))

        # Engineer button
        self.EngButton = QPushButton(Dialog)
        self.EngButton.setObjectName("Enginner Button")
        self.EngButton.setGeometry(QRect(240, 330, 201, 191))
        self.EngButton.setStyleSheet(Laning_buttons_st)
        icon2 = QIcon()
        icon2.addFile("Visuals/Engineer(final).png", QSize(), QIcon.Normal, QIcon.Off)
        self.EngButton.setIcon(icon2)
        self.EngButton.setIconSize(QSize(140, 140))

        # Float button
        self.FloatButton = QPushButton(Dialog)
        self.FloatButton.setObjectName("Float Button")
        self.FloatButton.setGeometry(QRect(489, 331, 201, 191))
        self.FloatButton.setStyleSheet(Laning_buttons_st)
        icon3 = QIcon()
        icon3.addFile("Visuals/Float(final).png", QSize(), QIcon.Normal, QIcon.Off)
        self.FloatButton.setIcon(icon3)
        self.FloatButton.setIconSize(QSize(160, 160))

        self.setText(Dialog)

        QMetaObject.connectSlotsByName(Dialog)

    def setText(self, Dialog):
        Dialog.setWindowTitle(QCoreApplication.translate("Dialog", "Dialog", None))
        self.Bg_label.setText("")
        self.PilotButton.setText("")
        self.CoButton.setText("")
        self.EngButton.setText("")
        self.FloatButton.setText("")

