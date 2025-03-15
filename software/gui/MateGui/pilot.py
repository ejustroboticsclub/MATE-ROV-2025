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

class PilotUi(object):
    def setupUi(self, Dialog):
        Dialog.setObjectName("Dialog")
        Dialog.resize(928, 596)
        QFontDatabase.addApplicationFont("Gill Sans.otf")
        font=QFont("Gill Sans",24)
        Afont=QFont("Gill Sans",13)

        # Background
        self.Bg_label = QLabel(Dialog)
        self.Bg_label.setObjectName("Background")
        self.Bg_label.setGeometry(QRect(-3, -5, 931, 601))
        self.Bg_label.setPixmap(QPixmap("Visuals/Background(final).jpg"))
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

        self.setText(Dialog)

        QMetaObject.connectSlotsByName(Dialog)

    def setText(self, Dialog):
        Dialog.setWindowTitle(QCoreApplication.translate("Dialog", "Dialog", None))
        self.Bg_label.setText("")
        self.BackButton.setText(QCoreApplication.translate("Dialog", "Back", None))
        self.CamButton.setText(QCoreApplication.translate("Dialog", "Launch Camera System", None))


   