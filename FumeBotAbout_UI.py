# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'FumeBotAbout_UI.ui'
#
# Created by: PyQt4 UI code generator 4.11.4
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

try:
    _encoding = QtGui.QApplication.UnicodeUTF8
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig)

class Ui_AboutDialog(object):
    def setupUi(self, AboutDialog):
        AboutDialog.setObjectName(_fromUtf8("AboutDialog"))
        AboutDialog.resize(560, 400)
        AboutDialog.setMinimumSize(QtCore.QSize(560, 400))
        AboutDialog.setMaximumSize(QtCore.QSize(560, 400))
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(_fromUtf8("bot_icon.jpg")), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        AboutDialog.setWindowIcon(icon)
        AboutDialog.setAutoFillBackground(False)
        AboutDialog.setModal(True)
        self.gridLayout = QtGui.QGridLayout(AboutDialog)
        self.gridLayout.setObjectName(_fromUtf8("gridLayout"))
        self.whole_verticalLayout = QtGui.QVBoxLayout()
        self.whole_verticalLayout.setObjectName(_fromUtf8("whole_verticalLayout"))
        self.image_horizontalLayout = QtGui.QHBoxLayout()
        self.image_horizontalLayout.setObjectName(_fromUtf8("image_horizontalLayout"))
        spacerItem = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.image_horizontalLayout.addItem(spacerItem)
        self.about_image_label = QtGui.QLabel(AboutDialog)
        self.about_image_label.setMaximumSize(QtCore.QSize(360, 240))
        self.about_image_label.setText(_fromUtf8(""))
        self.about_image_label.setPixmap(QtGui.QPixmap(_fromUtf8("about_image.jpg")))
        self.about_image_label.setScaledContents(True)
        self.about_image_label.setAlignment(QtCore.Qt.AlignCenter)
        self.about_image_label.setObjectName(_fromUtf8("about_image_label"))
        self.image_horizontalLayout.addWidget(self.about_image_label)
        spacerItem1 = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.image_horizontalLayout.addItem(spacerItem1)
        self.whole_verticalLayout.addLayout(self.image_horizontalLayout)
        spacerItem2 = QtGui.QSpacerItem(20, 40, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
        self.whole_verticalLayout.addItem(spacerItem2)
        self.aboutlabel = QtGui.QLabel(AboutDialog)
        self.aboutlabel.setObjectName(_fromUtf8("aboutlabel"))
        self.whole_verticalLayout.addWidget(self.aboutlabel)
        spacerItem3 = QtGui.QSpacerItem(20, 40, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
        self.whole_verticalLayout.addItem(spacerItem3)
        self.horizontalLayout = QtGui.QHBoxLayout()
        self.horizontalLayout.setObjectName(_fromUtf8("horizontalLayout"))
        spacerItem4 = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.horizontalLayout.addItem(spacerItem4)
        self.buttonBox = QtGui.QDialogButtonBox(AboutDialog)
        self.buttonBox.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.buttonBox.setAutoFillBackground(False)
        self.buttonBox.setOrientation(QtCore.Qt.Horizontal)
        self.buttonBox.setStandardButtons(QtGui.QDialogButtonBox.Ok)
        self.buttonBox.setCenterButtons(True)
        self.buttonBox.setObjectName(_fromUtf8("buttonBox"))
        self.horizontalLayout.addWidget(self.buttonBox)
        spacerItem5 = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.horizontalLayout.addItem(spacerItem5)
        self.whole_verticalLayout.addLayout(self.horizontalLayout)
        self.gridLayout.addLayout(self.whole_verticalLayout, 0, 0, 1, 1)

        self.retranslateUi(AboutDialog)
        QtCore.QMetaObject.connectSlotsByName(AboutDialog)

    def retranslateUi(self, AboutDialog):
        AboutDialog.setWindowTitle(_translate("AboutDialog", "About", None))
        self.aboutlabel.setText(_translate("AboutDialog", "<html><head/><body><p align=\"center\"><span style=\" font-size:10pt; font-weight:600;\">This application is used for controlling FumeBot an Arduino and</span></p><p align=\"center\"><span style=\" font-size:10pt; font-weight:600;\">Raspberry Pi based robot.</span></p><p align=\"center\"><span style=\" font-size:10pt; font-weight:600;\">Copyright © 2018 Ajith Thomas. All Rights Reserved.</span></p></body></html>", None))


if __name__ == "__main__":
    import sys
    app = QtGui.QApplication(sys.argv)
    AboutDialog = QtGui.QDialog()
    ui = Ui_AboutDialog()
    ui.setupUi(AboutDialog)
    AboutDialog.show()
    sys.exit(app.exec_())

