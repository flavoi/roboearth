/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created: Mon Nov 12 11:35:05 2012
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QMainWindow>
#include <QtGui/QPushButton>
#include <QtGui/QSpacerItem>
#include <QtGui/QTableWidget>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralwidget;
    QVBoxLayout *verticalLayout;
    QTableWidget *modelsTableWidget;
    QHBoxLayout *horizontalLayout;
    QPushButton *resendModelsButton;
    QSpacerItem *horizontalSpacer;
    QPushButton *addLocalModelButton;
    QPushButton *downloadModelButton;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(361, 598);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        verticalLayout = new QVBoxLayout(centralwidget);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        modelsTableWidget = new QTableWidget(centralwidget);
        if (modelsTableWidget->columnCount() < 3)
            modelsTableWidget->setColumnCount(3);
        QTableWidgetItem *__qtablewidgetitem = new QTableWidgetItem();
        modelsTableWidget->setHorizontalHeaderItem(0, __qtablewidgetitem);
        QTableWidgetItem *__qtablewidgetitem1 = new QTableWidgetItem();
        modelsTableWidget->setHorizontalHeaderItem(1, __qtablewidgetitem1);
        QTableWidgetItem *__qtablewidgetitem2 = new QTableWidgetItem();
        modelsTableWidget->setHorizontalHeaderItem(2, __qtablewidgetitem2);
        modelsTableWidget->setObjectName(QString::fromUtf8("modelsTableWidget"));
        modelsTableWidget->setEditTriggers(QAbstractItemView::NoEditTriggers);
        modelsTableWidget->setColumnCount(3);

        verticalLayout->addWidget(modelsTableWidget);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        resendModelsButton = new QPushButton(centralwidget);
        resendModelsButton->setObjectName(QString::fromUtf8("resendModelsButton"));

        horizontalLayout->addWidget(resendModelsButton);

        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer);

        addLocalModelButton = new QPushButton(centralwidget);
        addLocalModelButton->setObjectName(QString::fromUtf8("addLocalModelButton"));

        horizontalLayout->addWidget(addLocalModelButton);

        downloadModelButton = new QPushButton(centralwidget);
        downloadModelButton->setObjectName(QString::fromUtf8("downloadModelButton"));

        horizontalLayout->addWidget(downloadModelButton);


        verticalLayout->addLayout(horizontalLayout);

        MainWindow->setCentralWidget(centralwidget);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "Object Detector GUI", 0, QApplication::UnicodeUTF8));
        QTableWidgetItem *___qtablewidgetitem = modelsTableWidget->horizontalHeaderItem(0);
        ___qtablewidgetitem->setText(QApplication::translate("MainWindow", "File Path", 0, QApplication::UnicodeUTF8));
        QTableWidgetItem *___qtablewidgetitem1 = modelsTableWidget->horizontalHeaderItem(1);
        ___qtablewidgetitem1->setText(QApplication::translate("MainWindow", "Name", 0, QApplication::UnicodeUTF8));
        QTableWidgetItem *___qtablewidgetitem2 = modelsTableWidget->horizontalHeaderItem(2);
        ___qtablewidgetitem2->setText(QApplication::translate("MainWindow", "Type", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        resendModelsButton->setToolTip(QApplication::translate("MainWindow", "Send the model directories in the above table view to the detection methods.", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        resendModelsButton->setText(QApplication::translate("MainWindow", "Resend Model List", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        addLocalModelButton->setToolTip(QApplication::translate("MainWindow", "Add an already downloaded model to the model list above.", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        addLocalModelButton->setText(QApplication::translate("MainWindow", "Add Local Model", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        downloadModelButton->setToolTip(QApplication::translate("MainWindow", "Query and download object models from RoboEarth.", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        downloadModelButton->setText(QApplication::translate("MainWindow", "Download", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
