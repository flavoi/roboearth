/********************************************************************************
** Form generated from reading UI file 'downloadobjectdialog.ui'
**
** Created: Mon Nov 12 11:35:05 2012
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_DOWNLOADOBJECTDIALOG_H
#define UI_DOWNLOADOBJECTDIALOG_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QDialog>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QPushButton>
#include <QtGui/QSpacerItem>
#include <QtGui/QTableWidget>
#include <QtGui/QVBoxLayout>

QT_BEGIN_NAMESPACE

class Ui_DownloadObjectDialog
{
public:
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout;
    QLineEdit *searchEdit;
    QPushButton *searchButton;
    QLabel *queryFeedbackLbl;
    QTableWidget *resultsTableWidget;
    QHBoxLayout *horizontalLayout_3;
    QLabel *label;
    QLineEdit *downloadDirEdit;
    QPushButton *chooseDownloadDirButton;
    QHBoxLayout *horizontalLayout_2;
    QSpacerItem *horizontalSpacer;
    QPushButton *downloadButton;
    QPushButton *cancelButton;

    void setupUi(QDialog *DownloadObjectDialog)
    {
        if (DownloadObjectDialog->objectName().isEmpty())
            DownloadObjectDialog->setObjectName(QString::fromUtf8("DownloadObjectDialog"));
        DownloadObjectDialog->resize(397, 386);
        verticalLayout = new QVBoxLayout(DownloadObjectDialog);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        searchEdit = new QLineEdit(DownloadObjectDialog);
        searchEdit->setObjectName(QString::fromUtf8("searchEdit"));

        horizontalLayout->addWidget(searchEdit);

        searchButton = new QPushButton(DownloadObjectDialog);
        searchButton->setObjectName(QString::fromUtf8("searchButton"));

        horizontalLayout->addWidget(searchButton);


        verticalLayout->addLayout(horizontalLayout);

        queryFeedbackLbl = new QLabel(DownloadObjectDialog);
        queryFeedbackLbl->setObjectName(QString::fromUtf8("queryFeedbackLbl"));

        verticalLayout->addWidget(queryFeedbackLbl);

        resultsTableWidget = new QTableWidget(DownloadObjectDialog);
        if (resultsTableWidget->columnCount() < 4)
            resultsTableWidget->setColumnCount(4);
        QTableWidgetItem *__qtablewidgetitem = new QTableWidgetItem();
        __qtablewidgetitem->setTextAlignment(Qt::AlignLeft|Qt::AlignVCenter);
        resultsTableWidget->setHorizontalHeaderItem(0, __qtablewidgetitem);
        QTableWidgetItem *__qtablewidgetitem1 = new QTableWidgetItem();
        resultsTableWidget->setHorizontalHeaderItem(1, __qtablewidgetitem1);
        QTableWidgetItem *__qtablewidgetitem2 = new QTableWidgetItem();
        resultsTableWidget->setHorizontalHeaderItem(2, __qtablewidgetitem2);
        QTableWidgetItem *__qtablewidgetitem3 = new QTableWidgetItem();
        resultsTableWidget->setHorizontalHeaderItem(3, __qtablewidgetitem3);
        resultsTableWidget->setObjectName(QString::fromUtf8("resultsTableWidget"));
        resultsTableWidget->setEditTriggers(QAbstractItemView::NoEditTriggers);
        resultsTableWidget->setColumnCount(4);

        verticalLayout->addWidget(resultsTableWidget);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        label = new QLabel(DownloadObjectDialog);
        label->setObjectName(QString::fromUtf8("label"));

        horizontalLayout_3->addWidget(label);

        downloadDirEdit = new QLineEdit(DownloadObjectDialog);
        downloadDirEdit->setObjectName(QString::fromUtf8("downloadDirEdit"));

        horizontalLayout_3->addWidget(downloadDirEdit);

        chooseDownloadDirButton = new QPushButton(DownloadObjectDialog);
        chooseDownloadDirButton->setObjectName(QString::fromUtf8("chooseDownloadDirButton"));

        horizontalLayout_3->addWidget(chooseDownloadDirButton);


        verticalLayout->addLayout(horizontalLayout_3);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_2->addItem(horizontalSpacer);

        downloadButton = new QPushButton(DownloadObjectDialog);
        downloadButton->setObjectName(QString::fromUtf8("downloadButton"));

        horizontalLayout_2->addWidget(downloadButton);

        cancelButton = new QPushButton(DownloadObjectDialog);
        cancelButton->setObjectName(QString::fromUtf8("cancelButton"));

        horizontalLayout_2->addWidget(cancelButton);


        verticalLayout->addLayout(horizontalLayout_2);


        retranslateUi(DownloadObjectDialog);

        QMetaObject::connectSlotsByName(DownloadObjectDialog);
    } // setupUi

    void retranslateUi(QDialog *DownloadObjectDialog)
    {
        DownloadObjectDialog->setWindowTitle(QApplication::translate("DownloadObjectDialog", "Object Model Download", 0, QApplication::UnicodeUTF8));
        searchButton->setText(QApplication::translate("DownloadObjectDialog", "Search", 0, QApplication::UnicodeUTF8));
        queryFeedbackLbl->setText(QApplication::translate("DownloadObjectDialog", "Please enter a syntactic query in the field above and click \"Search\"", 0, QApplication::UnicodeUTF8));
        QTableWidgetItem *___qtablewidgetitem = resultsTableWidget->horizontalHeaderItem(0);
        ___qtablewidgetitem->setText(QApplication::translate("DownloadObjectDialog", "Selected", 0, QApplication::UnicodeUTF8));
        QTableWidgetItem *___qtablewidgetitem1 = resultsTableWidget->horizontalHeaderItem(1);
        ___qtablewidgetitem1->setText(QApplication::translate("DownloadObjectDialog", "UID", 0, QApplication::UnicodeUTF8));
        QTableWidgetItem *___qtablewidgetitem2 = resultsTableWidget->horizontalHeaderItem(2);
        ___qtablewidgetitem2->setText(QApplication::translate("DownloadObjectDialog", "Filename", 0, QApplication::UnicodeUTF8));
        QTableWidgetItem *___qtablewidgetitem3 = resultsTableWidget->horizontalHeaderItem(3);
        ___qtablewidgetitem3->setText(QApplication::translate("DownloadObjectDialog", "File URL", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("DownloadObjectDialog", "Download to:", 0, QApplication::UnicodeUTF8));
        chooseDownloadDirButton->setText(QApplication::translate("DownloadObjectDialog", "...", 0, QApplication::UnicodeUTF8));
        downloadButton->setText(QApplication::translate("DownloadObjectDialog", "Download Selected Models", 0, QApplication::UnicodeUTF8));
        cancelButton->setText(QApplication::translate("DownloadObjectDialog", "Cancel", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class DownloadObjectDialog: public Ui_DownloadObjectDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_DOWNLOADOBJECTDIALOG_H
