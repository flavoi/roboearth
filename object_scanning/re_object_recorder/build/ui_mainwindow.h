/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created: Mon Nov 12 11:33:37 2012
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
#include <QtGui/QDoubleSpinBox>
#include <QtGui/QFormLayout>
#include <QtGui/QGridLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QListWidget>
#include <QtGui/QPlainTextEdit>
#include <QtGui/QPushButton>
#include <QtGui/QSpinBox>
#include <QtGui/QTabWidget>
#include <QtGui/QTextEdit>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QAction *actionDelete;
    QGridLayout *gridLayout;
    QTabWidget *tabWidget;
    QWidget *recordTab;
    QVBoxLayout *verticalLayout;
    QFormLayout *formLayout_2;
    QLabel *boundingBoxSizeLabel;
    QDoubleSpinBox *boundingBoxSizeDoubleSpinBox;
    QLabel *maxScansLabel;
    QSpinBox *maxScansSpinBox;
    QPushButton *startstopbutton;
    QPushButton *resetButton;
    QPushButton *saveButton;
    QPushButton *loadButton;
    QLabel *label_5;
    QListWidget *cloudListWidget;
    QWidget *uploadTab;
    QVBoxLayout *verticalLayout_2;
    QFormLayout *formLayout;
    QLabel *label_3;
    QLineEdit *modelType;
    QLabel *label;
    QLineEdit *objectNameEdit;
    QLabel *label_2;
    QLineEdit *objectClassEdit;
    QLabel *label_4;
    QPlainTextEdit *objectDescriptionEdit;
    QLabel *label_7;
    QLineEdit *apiKeyEdit;
    QLabel *label_6;
    QTextEdit *owlView;
    QPushButton *reButton;
    QTextEdit *textEdit;

    void setupUi(QWidget *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(283, 544);
        actionDelete = new QAction(MainWindow);
        actionDelete->setObjectName(QString::fromUtf8("actionDelete"));
        gridLayout = new QGridLayout(MainWindow);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        tabWidget = new QTabWidget(MainWindow);
        tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
        recordTab = new QWidget();
        recordTab->setObjectName(QString::fromUtf8("recordTab"));
        verticalLayout = new QVBoxLayout(recordTab);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        formLayout_2 = new QFormLayout();
        formLayout_2->setObjectName(QString::fromUtf8("formLayout_2"));
        formLayout_2->setFieldGrowthPolicy(QFormLayout::ExpandingFieldsGrow);
        boundingBoxSizeLabel = new QLabel(recordTab);
        boundingBoxSizeLabel->setObjectName(QString::fromUtf8("boundingBoxSizeLabel"));
        boundingBoxSizeLabel->setLineWidth(1);
        boundingBoxSizeLabel->setTextFormat(Qt::AutoText);
        boundingBoxSizeLabel->setWordWrap(false);

        formLayout_2->setWidget(0, QFormLayout::LabelRole, boundingBoxSizeLabel);

        boundingBoxSizeDoubleSpinBox = new QDoubleSpinBox(recordTab);
        boundingBoxSizeDoubleSpinBox->setObjectName(QString::fromUtf8("boundingBoxSizeDoubleSpinBox"));
        boundingBoxSizeDoubleSpinBox->setDecimals(4);
        boundingBoxSizeDoubleSpinBox->setSingleStep(0.01);

        formLayout_2->setWidget(0, QFormLayout::FieldRole, boundingBoxSizeDoubleSpinBox);

        maxScansLabel = new QLabel(recordTab);
        maxScansLabel->setObjectName(QString::fromUtf8("maxScansLabel"));

        formLayout_2->setWidget(1, QFormLayout::LabelRole, maxScansLabel);

        maxScansSpinBox = new QSpinBox(recordTab);
        maxScansSpinBox->setObjectName(QString::fromUtf8("maxScansSpinBox"));
        maxScansSpinBox->setMinimum(1);
        maxScansSpinBox->setMaximum(500);
        maxScansSpinBox->setValue(100);

        formLayout_2->setWidget(1, QFormLayout::FieldRole, maxScansSpinBox);


        verticalLayout->addLayout(formLayout_2);

        startstopbutton = new QPushButton(recordTab);
        startstopbutton->setObjectName(QString::fromUtf8("startstopbutton"));
        startstopbutton->setCheckable(true);

        verticalLayout->addWidget(startstopbutton);

        resetButton = new QPushButton(recordTab);
        resetButton->setObjectName(QString::fromUtf8("resetButton"));

        verticalLayout->addWidget(resetButton);

        saveButton = new QPushButton(recordTab);
        saveButton->setObjectName(QString::fromUtf8("saveButton"));

        verticalLayout->addWidget(saveButton);

        loadButton = new QPushButton(recordTab);
        loadButton->setObjectName(QString::fromUtf8("loadButton"));

        verticalLayout->addWidget(loadButton);

        label_5 = new QLabel(recordTab);
        label_5->setObjectName(QString::fromUtf8("label_5"));

        verticalLayout->addWidget(label_5);

        cloudListWidget = new QListWidget(recordTab);
        cloudListWidget->setObjectName(QString::fromUtf8("cloudListWidget"));
        cloudListWidget->setSelectionMode(QAbstractItemView::ExtendedSelection);

        verticalLayout->addWidget(cloudListWidget);

        tabWidget->addTab(recordTab, QString());
        uploadTab = new QWidget();
        uploadTab->setObjectName(QString::fromUtf8("uploadTab"));
        verticalLayout_2 = new QVBoxLayout(uploadTab);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        formLayout = new QFormLayout();
        formLayout->setObjectName(QString::fromUtf8("formLayout"));
        formLayout->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
        label_3 = new QLabel(uploadTab);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        formLayout->setWidget(1, QFormLayout::LabelRole, label_3);

        modelType = new QLineEdit(uploadTab);
        modelType->setObjectName(QString::fromUtf8("modelType"));
        modelType->setReadOnly(true);

        formLayout->setWidget(1, QFormLayout::FieldRole, modelType);

        label = new QLabel(uploadTab);
        label->setObjectName(QString::fromUtf8("label"));

        formLayout->setWidget(2, QFormLayout::LabelRole, label);

        objectNameEdit = new QLineEdit(uploadTab);
        objectNameEdit->setObjectName(QString::fromUtf8("objectNameEdit"));

        formLayout->setWidget(2, QFormLayout::FieldRole, objectNameEdit);

        label_2 = new QLabel(uploadTab);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        formLayout->setWidget(3, QFormLayout::LabelRole, label_2);

        objectClassEdit = new QLineEdit(uploadTab);
        objectClassEdit->setObjectName(QString::fromUtf8("objectClassEdit"));

        formLayout->setWidget(3, QFormLayout::FieldRole, objectClassEdit);

        label_4 = new QLabel(uploadTab);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        formLayout->setWidget(4, QFormLayout::LabelRole, label_4);

        objectDescriptionEdit = new QPlainTextEdit(uploadTab);
        objectDescriptionEdit->setObjectName(QString::fromUtf8("objectDescriptionEdit"));

        formLayout->setWidget(4, QFormLayout::FieldRole, objectDescriptionEdit);

        label_7 = new QLabel(uploadTab);
        label_7->setObjectName(QString::fromUtf8("label_7"));

        formLayout->setWidget(0, QFormLayout::LabelRole, label_7);

        apiKeyEdit = new QLineEdit(uploadTab);
        apiKeyEdit->setObjectName(QString::fromUtf8("apiKeyEdit"));

        formLayout->setWidget(0, QFormLayout::FieldRole, apiKeyEdit);


        verticalLayout_2->addLayout(formLayout);

        label_6 = new QLabel(uploadTab);
        label_6->setObjectName(QString::fromUtf8("label_6"));

        verticalLayout_2->addWidget(label_6);

        owlView = new QTextEdit(uploadTab);
        owlView->setObjectName(QString::fromUtf8("owlView"));
        owlView->setReadOnly(false);

        verticalLayout_2->addWidget(owlView);

        reButton = new QPushButton(uploadTab);
        reButton->setObjectName(QString::fromUtf8("reButton"));

        verticalLayout_2->addWidget(reButton);

        tabWidget->addTab(uploadTab, QString());

        gridLayout->addWidget(tabWidget, 0, 0, 1, 1);

        textEdit = new QTextEdit(MainWindow);
        textEdit->setObjectName(QString::fromUtf8("textEdit"));
        textEdit->setMaximumSize(QSize(16777215, 100));
        textEdit->setReadOnly(true);

        gridLayout->addWidget(textEdit, 1, 0, 1, 1);


        retranslateUi(MainWindow);

        tabWidget->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QWidget *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "RoboEarth Object Recorder", 0, QApplication::UnicodeUTF8));
        actionDelete->setText(QApplication::translate("MainWindow", "Delete", 0, QApplication::UnicodeUTF8));
        boundingBoxSizeLabel->setText(QApplication::translate("MainWindow", "Bounding Box Size (in m):", 0, QApplication::UnicodeUTF8));
        maxScansLabel->setText(QApplication::translate("MainWindow", "Maximum number of scans:", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        maxScansSpinBox->setToolTip(QApplication::translate("MainWindow", "Limits the maximum number of recorded views.", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        startstopbutton->setText(QApplication::translate("MainWindow", "Start Recording", 0, QApplication::UnicodeUTF8));
        resetButton->setText(QApplication::translate("MainWindow", "Reset", 0, QApplication::UnicodeUTF8));
        saveButton->setText(QApplication::translate("MainWindow", "Save Model", 0, QApplication::UnicodeUTF8));
        loadButton->setText(QApplication::translate("MainWindow", "Import Model", 0, QApplication::UnicodeUTF8));
        label_5->setText(QApplication::translate("MainWindow", "Recorded Views:", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(recordTab), QApplication::translate("MainWindow", "Recording", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("MainWindow", "Model Type:", 0, QApplication::UnicodeUTF8));
        modelType->setText(QApplication::translate("MainWindow", "ColoredPointCloudModel", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("MainWindow", "Object Name:", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("MainWindow", "Object Class:", 0, QApplication::UnicodeUTF8));
        label_4->setText(QApplication::translate("MainWindow", "Description:", 0, QApplication::UnicodeUTF8));
        label_7->setText(QApplication::translate("MainWindow", "API Key:", 0, QApplication::UnicodeUTF8));
        label_6->setText(QApplication::translate("MainWindow", "OWL Description:", 0, QApplication::UnicodeUTF8));
        reButton->setText(QApplication::translate("MainWindow", "Upload to RoboEarth", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(uploadTab), QApplication::translate("MainWindow", "Uploading", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
