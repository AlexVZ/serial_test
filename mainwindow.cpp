/****************************************************************************
**
** Copyright (C) 2014 Alex
**
** $QT_BEGIN_LICENSE:BSD$
** You may use this file under the terms of the BSD license as follows:
**
** "Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are
** met:
**   * Redistributions of source code must retain the above copyright
**     notice, this list of conditions and the following disclaimer.
**   * Redistributions in binary form must reproduce the above copyright
**     notice, this list of conditions and the following disclaimer in
**     the documentation and/or other materials provided with the
**     distribution.
**   * Neither the name of Digia Plc and its Subsidiary(-ies) nor the names
**     of its contributors may be used to endorse or promote products derived
**     from this software without specific prior written permission.
**
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
**
** $QT_END_LICENSE$
**
****************************************************************************/

#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    m_serial_port(NULL)
{
    ui->setupUi(this);

    fillPortsCombo();
    fillSpeedsCombo();

    connect(ui->buttonOpen, SIGNAL(clicked(bool)), this, SLOT(buttonOpen_clicked(bool)));
    connect(ui->buttonClose, SIGNAL(clicked(bool)), this, SLOT(buttonClose_clicked(bool)));
    connect(ui->buttonSend, SIGNAL(clicked(bool)), this, SLOT(buttonSend_clicked(bool)));
    connect(ui->buttonClearReceived, SIGNAL(clicked(bool)), this, SLOT(buttonClearReceived_clicked(bool)));

    setButtonsEnabled(false);

    m_serial_port = new QSerialPort(this);
    connect(m_serial_port, SIGNAL(error(QSerialPort::SerialPortError)), this, SLOT(serial_error_handler(QSerialPort::SerialPortError)));
    connect(m_serial_port, SIGNAL(readyRead()), this, SLOT(read_port()));

    ui->textToSend->setText(tr("BA 02 01 B9"));
    ui->statusBar->setText(tr("ready"));
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::fillPortsCombo()
{
    foreach(const QSerialPortInfo &port_info, QSerialPortInfo::availablePorts()) {
        ui->portName->addItem(port_info.systemLocation());
        //ui->portName->addItem(port_info.portName());
    }
}

void MainWindow::fillSpeedsCombo()
{
    ui->portSpeed->addItem(tr("1200"));
    ui->portSpeed->addItem(tr("2400"));
    ui->portSpeed->addItem(tr("4800"));
    ui->portSpeed->addItem(tr("9600"));
    ui->portSpeed->addItem(tr("14400"));
    ui->portSpeed->addItem(tr("19200"));
    ui->portSpeed->addItem(tr("38400"));
    ui->portSpeed->addItem(tr("57600"));
    ui->portSpeed->addItem(tr("115200"));
    ui->portSpeed->addItem(tr("230400"));
    ui->portSpeed->addItem(tr("460800"));
    ui->portSpeed->setCurrentIndex(8);
}

void MainWindow::setButtonsEnabled(bool is_port_opened)
{
    if(is_port_opened) {
        ui->portName->setEnabled(false);
        ui->portSpeed->setEnabled(false);
        ui->buttonClose->setEnabled(true);
        ui->buttonOpen->setEnabled(false);
        ui->textToSend->setEnabled(true);
        ui->buttonSend->setEnabled(true);
    } else {
        ui->portName->setEnabled(true);
        ui->portSpeed->setEnabled(true);
        ui->buttonClose->setEnabled(false);
        ui->buttonOpen->setEnabled(true);
        ui->textToSend->setEnabled(false);
        ui->buttonSend->setEnabled(false);
    }
}

void MainWindow::buttonOpen_clicked(bool)
{
    QString port_name = ui->portName->currentText();
    QString port_speed = ui->portSpeed->currentText();
    if(open_port(port_name, port_speed.toInt())) {
        ui->statusBar->setText(tr("serial port opened"));
        setButtonsEnabled(true);
    } else {
        ui->statusBar->setText(tr("serial port (") + port_name + tr(") NOT opened (") + m_serial_port->errorString() + tr(")"));
        //ui->statusBar->setText(m_serial_port->errorString());
    }
}
void MainWindow::buttonClose_clicked(bool)
{
    close_port();
    ui->textSended->setPlainText(tr(""));
    ui->textReceived->setPlainText(tr(""));
    ui->statusBar->setText(tr("serial port closed"));
    setButtonsEnabled(false);
}

void MainWindow::buttonSend_clicked(bool)
{
    //unsigned char atr_bytes[] = {0x3B, 0x9E, 0x95, 0x80, 0x1F, 0xC3, 0x80, 0x31, 0xA0, 0x73, 0xBE, 0x21, 0x13, 0x67, 0x29, 0x02, 0x01, 0x01, 0x81,0xCD,0xB9};
//unsigned char atr_bytes[] = {0xAA, 0xBB, 0x06, 0x00, 0x00, 0x00, 0x06, 0x01, 0x01, 0x06}; // SL500; 10ms beep
//unsigned char atr_bytes[] = {0xAA, 0xBB, 0x06, 0x00, 0x00, 0x00, 0x06, 0x01, 0x0A, 0x0D}; // SL500; 100ms beep
//unsigned char atr_bytes[] = {0xAA, 0xBB, 0x06, 0x00, 0x00, 0x00, 0x07, 0x01, 0x02, 0x00}; // SL500; led color green
//unsigned char atr_bytes[] = {0xAA, 0xBB, 0x06, 0x00, 0x00, 0x00, 0x0C, 0x01, 0x01, 0x00}; // SL500; transmit on
//unsigned char atr_bytes[] = {0xAA, 0xBB, 0x05, 0x00, 0x00, 0x00, 0x04, 0x01, 0x00}; // SL500; model and product number
//unsigned char atr_bytes[] = {0xAA, 0xBB, 0x06, 0x00, 0x00, 0x00, 0x01, 0x02, 0x26, 0x00}; // SL500; card type code
//unsigned char atr_bytes[] = {0xAA, 0xBB, 0x06, 0x00, 0x00, 0x00, 0x01, 0x02, 0x52, 0x00}; // SL500; card type code
//unsigned char atr_bytes[] = {0xAA, 0xBB, 0x06, 0x00, 0x00, 0x00, 0x08, 0x01, 0x0A, 0x00}; // SL500; TYPE_A mode
//unsigned char atr_bytes[] = {0xAA, 0xBB, 0x06, 0x00, 0x00, 0x00, 0x08, 0x01, 0x0B, 0x00}; // SL500; TYPE_B mode
//unsigned char atr_bytes[] = {0xAA, 0xBB, 0x06, 0x00, 0x00, 0x00, 0x08, 0x01, 0x01, 0x00}; // SL500; ISO15693 mode
//unsigned char atr_bytes[] = {0xAA, 0xBB, 0x06, 0x00, 0x00, 0x00, 0x08, 0x02, 0x00, 0x00}; // SL500; read start from 0x00
//unsigned char atr_bytes[] = {0xAA, 0xBB, 0x06, 0x00, 0x00, 0x00, 0x10, 0x02, 0x26, 0x00}; // SL500; request MifareProX and reset
//unsigned char atr_bytes[] = {0xAA, 0xBB, 0x05, 0x00, 0x00, 0x00, 0x02, 0x02, 0x00}; // SL500; card serial number
//unsigned char atr_bytes[] = {0xAA, 0xBB, 0x05, 0x00, 0x00, 0x00, 0x01, 0x10, 0x00}; // SL500;
/* last byte is crc:
int arr_size = sizeof(atr_bytes)/sizeof(atr_bytes[0]);
unsigned char crc = 0;
for(int i = 4; i < arr_size-1; i++) {
    crc ^= atr_bytes[i];
}
atr_bytes[arr_size-1] = crc;
*/
/*
unsigned char atr_bytes[] = {0xBA, 0x02, 0x01, 0xB9};

    QByteArray byte_arr((char *)atr_bytes, sizeof(atr_bytes)/sizeof(atr_bytes[0]));
    ui->textSended->setPlainText(byte_array_to_string(byte_arr));
    ui->textReceived->appendPlainText(tr("\n"));
    qint64 writed = write_port(byte_arr);
    ui->statusBar->setText(tr("ATR sended (") + QString::number(writed) + tr(" bytes writed)"));
*/
// BA 02 01 B9
    QByteArray byte_arr = byte_array_from_string(ui->textToSend->text());

    if(byte_arr.size() == 9 || byte_arr.size() == 10) {
        int arr_size = byte_arr.size();
        unsigned char crc = 0;
        for(int i = 4; i < arr_size-1; i++) {
            crc ^= byte_arr[i];
        }
        byte_arr[arr_size-1] = crc;
    }

    ui->textSended->setPlainText(byte_array_to_string(byte_arr));
    //ui->textReceived->appendPlainText(tr("\n"));
    qint64 writed = write_port(byte_arr);
    if(writed <= 0) {
        ui->statusBar->setText(tr("fail; ") + QString::number(writed) + tr(" bytes writed;"));
    } else if(writed == byte_arr.length()) {
        ui->statusBar->setText(tr("ok; ") + QString::number(writed) + tr(" bytes writed;"));
    } else {
        ui->statusBar->setText(tr("partial writed: ") + QString::number(writed) + tr(" bytes writed from ") + QString::number(byte_arr.length()) + tr(";"));
    }
}

void MainWindow::buttonClearReceived_clicked(bool)
{
    if(ui->textReceived->toPlainText().length() == 0) {
        ui->statusBar->setText(tr("nothing to do"));
    } else {
        ui->textReceived->setPlainText(tr(""));
        ui->statusBar->setText(tr("received cleared"));
    }
}

void MainWindow::serial_error_handler(QSerialPort::SerialPortError serial_error)
{
    if(serial_error != QSerialPort::NoError) {
        ui->statusBar->setText(tr("serial port error: ") + m_serial_port->errorString());
    }
}

bool MainWindow::open_port(QString port_name, int port_speed)
{
    if(m_serial_port) {
        close_port();
    }
    //m_serial_port = new QSerialPort(this);
    m_serial_port->setPortName(port_name);
    m_serial_port->setBaudRate(port_speed);

    m_serial_port->setDataBits(QSerialPort::Data8);
    m_serial_port->setParity(QSerialPort::NoParity);
    m_serial_port->setStopBits(QSerialPort::OneStop);
    m_serial_port->setFlowControl(QSerialPort::NoFlowControl);

    if(m_serial_port->open(QIODevice::ReadWrite)) {
        //connect(m_serial_port, SIGNAL(error(QSerialPort::SerialPortError)), this, SLOT(serial_error_handler(QSerialPort::SerialPortError)));
        //connect(m_serial_port, SIGNAL(readyRead()), this, SLOT(read_port()));
        return true;
    } else {
        //delete m_serial_port;
        //m_serial_port = NULL;
        return false;
    }
}

void MainWindow::close_port()
{
    if(m_serial_port) {
        //disconnect(m_serial_port, SIGNAL(error(QSerialPort::SerialPortError)), this, SLOT(serial_error_handler(QSerialPort::SerialPortError)));
        //disconnect(m_serial_port, SIGNAL(readyRead()), this, SLOT(read_port()));
        m_serial_port->close();
        //delete m_serial_port;
        //m_serial_port = NULL;
    }
}

qint64 MainWindow::write_port(QByteArray &byte_arr)
{
    qint64 writed = m_serial_port->write(byte_arr.constData(), byte_arr.length());
    return writed;
}
void MainWindow::read_port()
{
    QByteArray byte_arr = m_serial_port->readAll();
    ui->textReceived->appendPlainText(byte_array_to_string(byte_arr));
    QString status_text = ui->statusBar->text();
    status_text += tr(" ") + QString::number(byte_arr.length()) + tr(" bytes received;");
    ui->statusBar->setText(status_text);
}

QString MainWindow::byte_array_to_string(QByteArray &byte_arr)
{
    QString result = tr("");
    for(int i = 0; i < byte_arr.length(); i++) {
        if(i > 0) {
            result += tr(" ");
        }
        char cur_byte_ch = byte_arr.at(i);
        QByteArray cur_byte(&cur_byte_ch, 1);
        result += cur_byte.toHex();
    }
    result = result.toUpper();
    return result;
}
QByteArray MainWindow::byte_array_from_string(QString hex_string)
{
    QStringList hex_list = hex_string.split(tr(" "));
    QByteArray result;
    for(int i = 0; i < hex_list.length(); i++) {
        QByteArray text = QByteArray::fromHex(hex_list[i].toLatin1());
        result.append(text);
    }
    return result;
}
