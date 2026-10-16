#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QDateTime>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFontDatabase>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QMainWindow>
#include <QMessageBox>
#include <QNetworkDatagram>
#include <QNetworkInterface>
#include <QProcess>
#include <QPushButton>
#include <QSettings>
#include <QScrollArea>
#include <QSocketNotifier>
#include <QSpinBox>
#include <QTableWidget>
#include <QTextEdit>
#include <QTimer>
#include <QToolButton>
#include <QVBoxLayout>
#include <QWidget>
#include <QTcpSocket>
#include <QUdpSocket>
#include <QtMath>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#ifdef Q_OS_LINUX
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

extern "C" {
#include "example_soem/motor_layer.h"
}

namespace {

struct MotorEntry {
    int id = 0;
    bool enabled = true;
    QString name;
    MotorModel model = MOTOR_MODEL_EC_A2806_P2;
    QString commandName = "spd";
    double param1 = 1.0;
    double param2 = 1.5;
    double param3 = 0.0;
    double param4 = 0.0;
    double target = 0.0;
    double speed = 0.0;
    double current = 0.0;
    double torque = 0.0;
    std::array<uint8_t, 8> rawData{};
    int rawDataLen = 2;
    bool rawPayloadManual = false;
};

struct MotorFeedbackState {
    MotorPackMsg packet{};
    MotorFeedback1 decoded{};
    QDateTime receivedAt;
    bool received = false;
};

constexpr uint8_t kFeedbackType1 = 1;

int commandDataLength(const QString& commandName) {
    if (commandName == "pos") {
        return 8;
    }
    if (commandName == "spd") {
        return 7;
    }
    if (commandName == "cur" || commandName == "tor" || commandName == "stop") {
        return 3;
    }
    if (commandName == "set-id" || commandName == "set-pos") {
        return 6;
    }
    return 8;
}

QString formatHexByte(uint8_t value) {
    return QString("%1").arg(value, 2, 16, QChar('0')).toUpper();
}

std::array<uint8_t, 8> encodePackedPosition(float position) {
    uint32_t positionBits = 0;
    std::memcpy(&positionBits, &position, sizeof(positionBits));

    constexpr uint64_t mode = 1;
    constexpr uint64_t speed = 500;
    constexpr uint64_t current = 100;
    constexpr uint64_t ack = 1;
    const uint64_t frame = ((mode & 0x07u) << 61) |
                           (static_cast<uint64_t>(positionBits) << 29) |
                           ((speed & 0x7FFFu) << 14) |
                           ((current & 0x0FFFu) << 2) | (ack & 0x03u);

    std::array<uint8_t, 8> payload{};
    for (size_t i = 0; i < payload.size(); ++i) {
        payload[i] = static_cast<uint8_t>(frame >> (56 - (i * 8)));
    }
    return payload;
}

std::array<uint8_t, 7> encodeSpeedCommand(float speed) {
    speed = std::round(speed * 1000.0f) / 1000.0f;
    if (std::fabs(speed) < 0.0005f) {
        speed = 0.0f;
    }

    uint32_t speedBits = 0;
    std::memcpy(&speedBits, &speed, sizeof(speedBits));
    return {{0x41,
             static_cast<uint8_t>(speedBits >> 24),
             static_cast<uint8_t>(speedBits >> 16),
             static_cast<uint8_t>(speedBits >> 8),
             static_cast<uint8_t>(speedBits),
             0x00,
             0x64}};
}

enum class TransportMode {
    UdpCan,
    TcpTcp,
    CanSocketCan
};

void applyDefaultRawData(MotorEntry& motor) {
    std::fill(motor.rawData.begin(), motor.rawData.end(), static_cast<uint8_t>(0x00));
    motor.rawDataLen = commandDataLength(motor.commandName);
    motor.rawPayloadManual = false;

    if (motor.commandName == "spd") {
        const auto payload = encodeSpeedCommand(0.0f);
        std::copy(payload.begin(), payload.end(), motor.rawData.begin());
        motor.rawPayloadManual = true;
    } else if (motor.commandName == "pos") {
        motor.rawData = encodePackedPosition(0.0f);
        motor.rawPayloadManual = true;
    } else if (motor.commandName == "cur") {
        motor.rawData[0] = 0x60;
        motor.rawData[1] = 0x00;
        motor.rawData[2] = 0x00;
    } else if (motor.commandName == "tor") {
        motor.rawData[0] = 0x64;
        motor.rawData[1] = 0x00;
        motor.rawData[2] = 0x00;
    } else if (motor.commandName == "stop") {
        motor.rawData[0] = 0x00;
        motor.rawData[1] = 0x00;
        motor.rawData[2] = 0x00;
    } else if (motor.commandName == "set-pos") {
        motor.rawData[0] = 0x00;
        motor.rawData[1] = 0x00;
        motor.rawData[2] = 0x00;
        motor.rawData[3] = 0x03;
        motor.rawData[4] = 0x00;
        motor.rawData[5] = 0x00;
    } else if (motor.commandName == "set-id") {
        motor.rawData[0] = 0x00;
        motor.rawData[1] = 0x00;
        motor.rawData[2] = 0x00;
        motor.rawData[3] = 0x04;
        motor.rawData[4] = 0x00;
        motor.rawData[5] = 0x00;
    }
}

QByteArray buildMotorPacketBytes(const MotorEntry& motor, const QString& commandName,
                                double param1, double param2, double param3, double param4) {
    MotorConfig cfg = motor_config_make(1, 0, static_cast<uint16_t>(motor.id), motor.model);
    MotorPackMsg packet{};

    if (motor.rawPayloadManual) {
        packet.id = static_cast<uint32_t>(motor.id);
        packet.frame_flags = 0x00;
        packet.len = static_cast<uint8_t>(motor.rawDataLen);
        std::memset(packet.data, 0, sizeof(packet.data));
        for (int i = 0; i < motor.rawDataLen; ++i) {
            packet.data[i] = motor.rawData[i];
        }
    } else if (commandName == "spd") {
        motor_build_spd_control(&cfg, static_cast<float>(param1), static_cast<float>(param2),
                                kFeedbackType1, &packet);
    } else if (commandName == "pos") {
        motor_build_pos_control(&cfg, static_cast<float>(param1), static_cast<float>(param2),
                                static_cast<float>(param3), kFeedbackType1, &packet);
    } else if (commandName == "cur") {
        motor_build_cur_control(&cfg, static_cast<float>(param1), static_cast<uint8_t>(param2),
                                &packet);
    } else if (commandName == "tor") {
        motor_build_tor_control(&cfg, static_cast<float>(param1), static_cast<uint8_t>(param2),
                                &packet);
    } else if (commandName == "stop") {
        motor_build_stop(&cfg, MOTOR_STOP_FULL_BRAKE, static_cast<float>(param1),
                        static_cast<uint8_t>(param2), &packet);
    } else if (commandName == "set-pos") {
        motor_build_set_pos(&cfg, static_cast<float>(param1), &packet);
    } else if (commandName == "set-id") {
        motor_build_set_id(&cfg, static_cast<uint16_t>(param1), &packet);
    } else {
        motor_build_spd_control(&cfg, 0.0f, 0.0f, 0, &packet);
    }

    QByteArray out;
    out.resize(static_cast<int>(sizeof(packet)));
    std::memcpy(out.data(), &packet, sizeof(packet));
    return out;
}

QString packetToText(const QByteArray& bytes) {
    QString out;
    if (bytes.size() < static_cast<int>(sizeof(MotorPackMsg))) {
        return "invalid packet";
    }

    MotorPackMsg packet{};
    std::memcpy(&packet, bytes.constData(), sizeof(packet));

    out = QString("CAN id=%1 flags=0x%2 len=%3 data=")
              .arg(packet.id)
              .arg(static_cast<int>(packet.frame_flags), 2, 16, QChar('0'))
              .arg(static_cast<int>(packet.len));

    for (uint8_t i = 0; i < packet.len; ++i) {
        out += QString("%1 ").arg(static_cast<int>(packet.data[i]), 2, 16, QChar('0'));
    }
    return out;
}

QString feedbackFrameToText(const MotorPackMsg& packet) {
    QString out = QString("0x%1 [").arg(packet.id, 3, 16, QChar('0')).toUpper();
    const uint8_t length = packet.len > sizeof(packet.data)
                               ? static_cast<uint8_t>(sizeof(packet.data))
                               : packet.len;
    for (uint8_t i = 0; i < length; ++i) {
        if (i != 0) {
            out += ' ';
        }
        out += formatHexByte(packet.data[i]);
    }
    out += ']';
    return out;
}

}  // namespace

class MotorControlWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MotorControlWindow(QWidget* parent = nullptr)
        : QMainWindow(parent),
          frameTimer_(new QTimer(this)),
          feedbackTimer_(new QTimer(this)),
          autoSendTimer_(new QTimer(this)),
          networkCheckTimer_(new QTimer(this)),
          calibrationTimer_(new QTimer(this)),
          udpSocket_(new QUdpSocket(this)),
          tcpSocket_(new QTcpSocket(this)),
          canSocket_(-1) {
        setWindowTitle("Robot Motor Control Dashboard");
        resize(1200, 760);
        setWindowFlags(windowFlags() | Qt::WindowMaximizeButtonHint);
        initializeMotors();
        buildUi();
        connect(frameTimer_, &QTimer::timeout, this, &MotorControlWindow::onFrameRefreshTick);
        connect(feedbackTimer_, &QTimer::timeout, this, &MotorControlWindow::onFeedbackRefreshTick);
        connect(autoSendTimer_, &QTimer::timeout, this, &MotorControlWindow::onAutoSendTick);
        connect(networkCheckTimer_, &QTimer::timeout, this, &MotorControlWindow::onNetworkCheck);
        calibrationTimer_->setSingleShot(true);
        connect(calibrationTimer_, &QTimer::timeout, this,
                &MotorControlWindow::onCalibrationTimeout);
        connect(udpSocket_, &QUdpSocket::readyRead, this, &MotorControlWindow::onUdpReadyRead);
        connect(tcpSocket_, &QTcpSocket::readyRead, this, &MotorControlWindow::onTcpReadyRead);
        frameTimer_->start(500);
        feedbackTimer_->start(1000);
        networkCheckTimer_->start(1000);
        refreshLocalAddressInfo();
        updateFramePreview();
        updateFeedbackTable();
    }

private slots:
    void onSendClicked() {
        sendCurrentFrame();
    }

    void onStopSendingClicked() {
        autoSendTimer_->stop();
        calibrationResumeAutoSend_ = false;
    }

    void onSelectAllClicked() {
        for (int i = 0; i < motorList_.size(); ++i) {
            motorList_[i].enabled = true;
            if (motorMatrix_ && motorMatrix_->cellWidget(i, 0)) {
                auto* cb = qobject_cast<QCheckBox*>(motorMatrix_->cellWidget(i, 0));
                if (cb) {
                    cb->setChecked(true);
                }
            }
        }
        updateFramePreview();
    }

    void onClearAllClicked() {
        for (int i = 0; i < motorList_.size(); ++i) {
            motorList_[i].enabled = false;
            if (motorMatrix_ && motorMatrix_->cellWidget(i, 0)) {
                auto* cb = qobject_cast<QCheckBox*>(motorMatrix_->cellWidget(i, 0));
                if (cb) {
                    cb->setChecked(false);
                }
            }
            applyMotorRowVisualState(i, "idle");
        }
        updateFramePreview();
    }

    void onSetDirectionClicked() {
        const std::array<int, 4> motorIds = {{8, 2, 6, 4}};
        const std::array<QDoubleSpinBox*, 4> positionEditors = {{
            positionCan8Edit_, positionCan2Edit_, positionCan6Edit_, positionCan4Edit_}};

        for (size_t i = 0; i < motorIds.size(); ++i) {
            const int row = motorIds[i] - 1;
            MotorEntry& motor = motorList_[row];
            motor.enabled = true;

            if (auto* enableBox = qobject_cast<QCheckBox*>(motorMatrix_->cellWidget(row, 0))) {
                enableBox->setChecked(true);
            }
            if (auto* command = qobject_cast<QComboBox*>(motorMatrix_->cellWidget(row, 3))) {
                command->setCurrentText("pos");
            }

            motor.commandName = "pos";
            motor.rawData = encodePackedPosition(static_cast<float>(positionEditors[i]->value()));
            motor.rawDataLen = static_cast<int>(motor.rawData.size());
            motor.rawPayloadManual = true;
            refreshHexByteEditorsForRow(row);
        }

        updateFramePreview();
    }

    void onSetSpeedClicked() {
        const std::array<int, 4> motorIds = {{7, 1, 5, 3}};
        const std::array<QDoubleSpinBox*, 4> speedEditors = {{
            speedCan7Edit_, speedCan1Edit_, speedCan5Edit_, speedCan3Edit_}};

        for (size_t i = 0; i < motorIds.size(); ++i) {
            const int row = motorIds[i] - 1;
            MotorEntry& motor = motorList_[row];
            motor.enabled = true;
            if (auto* enableBox = qobject_cast<QCheckBox*>(motorMatrix_->cellWidget(row, 0))) {
                enableBox->setChecked(true);
            }
            if (auto* command = qobject_cast<QComboBox*>(motorMatrix_->cellWidget(row, 3))) {
                command->setCurrentText("spd");
            }

            motor.commandName = "spd";
            motor.rawData.fill(0);
            const auto payload = encodeSpeedCommand(static_cast<float>(speedEditors[i]->value()));
            std::copy(payload.begin(), payload.end(), motor.rawData.begin());
            motor.rawDataLen = static_cast<int>(payload.size());
            motor.rawPayloadManual = true;
            refreshHexByteEditorsForRow(row);
        }

        updateFramePreview();
    }

    void onArrowUpClicked() {
        speedCan7Edit_->setValue(speedCan7Edit_->value() + 0.5);
        speedCan5Edit_->setValue(speedCan5Edit_->value() + 0.5);
        speedCan1Edit_->setValue(speedCan1Edit_->value() - 0.5);
        speedCan3Edit_->setValue(speedCan3Edit_->value() - 0.5);
        onSetSpeedClicked();
    }

    void onArrowDownClicked() {
        speedCan7Edit_->setValue(speedCan7Edit_->value() - 0.5);
        speedCan5Edit_->setValue(speedCan5Edit_->value() - 0.5);
        speedCan1Edit_->setValue(speedCan1Edit_->value() + 0.5);
        speedCan3Edit_->setValue(speedCan3Edit_->value() + 0.5);
        onSetSpeedClicked();
    }

    void onArrowLeftClicked() {
        positionCan8Edit_->setValue(positionCan8Edit_->value() - 0.5);
        positionCan2Edit_->setValue(positionCan2Edit_->value() - 0.5);
        positionCan6Edit_->setValue(positionCan6Edit_->value() + 0.5);
        positionCan4Edit_->setValue(positionCan4Edit_->value() + 0.5);
        onSetDirectionClicked();
    }

    void onArrowRightClicked() {
        positionCan8Edit_->setValue(positionCan8Edit_->value() + 0.5);
        positionCan2Edit_->setValue(positionCan2Edit_->value() + 0.5);
        positionCan6Edit_->setValue(positionCan6Edit_->value() - 0.5);
        positionCan4Edit_->setValue(positionCan4Edit_->value() - 0.5);
        onSetDirectionClicked();
    }

    void onCalibrationClicked() {
        if (!networkConnected_) {
            if (transportCombo_->currentData().toInt() ==
                    static_cast<int>(TransportMode::CanSocketCan) &&
                !applySocketCanBitrate()) {
                QMessageBox::warning(
                    this, "Calibration",
                    QString("Could not prepare SocketCAN interface %1.")
                        .arg(canInterfaceEdit_->text().trimmed()));
                return;
            }
            if (!probeNetwork()) {
                QMessageBox::warning(this, "Calibration",
                                     "Could not connect the selected transport.");
                return;
            }
            networkConnected_ = true;
            setNetworkStatus(true, "CONNECTED");
            connectButton_->setChecked(true);
            connectButton_->setText("Disconnect Network");
        } else if (!probeNetwork()) {
            QMessageBox::warning(this, "Calibration",
                                 "The selected transport is no longer connected.");
            return;
        }

        calibrationResumeAutoSend_ = autoSendTimer_->isActive();
        autoSendTimer_->stop();
        calibrationPending_.fill(true);
        calibrationActive_ = true;
        calibrationButton_->setEnabled(false);
        calibrationButton_->setText("Reading...");

        std::array<MotorPackMsg, 8> queries{};
        QByteArray payload;
        for (int motorId = 1; motorId <= 8; ++motorId) {
            MotorConfig config = motor_config_make(
                1, 0, static_cast<uint16_t>(motorId), MOTOR_MODEL_EC_A2806_P2);
            const MotorParameter parameter = ((motorId % 2) != 0)
                                                 ? MOTOR_PARAM_SPEED
                                                 : MOTOR_PARAM_POSITION;
            if (!motor_build_get_parameter(
                    &config, parameter, &queries[static_cast<size_t>(motorId - 1)])) {
                finishCalibration(false, "Failed to build query frames.");
                return;
            }
            payload.append(
                reinterpret_cast<const char*>(&queries[static_cast<size_t>(motorId - 1)]),
                static_cast<int>(sizeof(MotorPackMsg)));
        }

        bool sent = false;
        const int transport = transportCombo_->currentData().toInt();
        if (transport == static_cast<int>(TransportMode::CanSocketCan)) {
            int sentCount = 0;
            sent = true;
            for (const MotorPackMsg& query : queries) {
                if (!sendCanFrame(query, sentCount)) {
                    sent = false;
                    break;
                }
            }
        } else if (transport == static_cast<int>(TransportMode::UdpCan)) {
            const qint64 count = udpSocket_->writeDatagram(
                payload, QHostAddress(hostEdit_->text()), portEdit_->value());
            sent = count == payload.size();
        } else {
            if (tcpSocket_->state() != QAbstractSocket::ConnectedState) {
                tcpSocket_->connectToHost(hostEdit_->text(), portEdit_->value());
            }
            if (tcpSocket_->state() == QAbstractSocket::ConnectedState ||
                tcpSocket_->waitForConnected(1000)) {
                const qint64 count = tcpSocket_->write(payload);
                sent = count == payload.size() && tcpSocket_->waitForBytesWritten(1000);
            }
        }

        if (!sent) {
            finishCalibration(false, "Failed to send the eight query frames.");
            return;
        }

        calibrationTimer_->start(1500);
    }

    void onCalibrationTimeout() {
        QStringList missing;
        for (size_t index = 0; index < calibrationPending_.size(); ++index) {
            if (calibrationPending_[index]) {
                missing << QString::number(index + 1);
            }
        }
        finishCalibration(false,
                          QString("No query response from CAN %1.").arg(missing.join(", ")));
    }

    void onStopClicked() {
        for (int row = 0; row < motorList_.size(); ++row) {
            MotorEntry& motor = motorList_[row];
            motor.enabled = true;
            if (auto* enableBox = qobject_cast<QCheckBox*>(motorMatrix_->cellWidget(row, 0))) {
                enableBox->setChecked(true);
            }
            if (auto* command = qobject_cast<QComboBox*>(motorMatrix_->cellWidget(row, 3))) {
                command->setCurrentText("stop");
            }

            motor.commandName = "stop";
            motor.rawData.fill(0);
            motor.rawData[0] = 0x69;
            motor.rawDataLen = 3;
            motor.rawPayloadManual = true;
            refreshHexByteEditorsForRow(row);
        }

        updateFramePreview();
    }

    void onSaveFeedbackClicked() {
        const QString fileName = QFileDialog::getSaveFileName(this, "Save feedback log", "motor_feedback.txt",
                                                              "Text files (*.txt);;CSV files (*.csv);;All files (*.*)");
        if (fileName.isEmpty()) {
            return;
        }

        QFile file(fileName);
        if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
            QMessageBox::critical(this, "Save failed", "Unable to open file for writing.");
            return;
        }

        QTextStream out(&file);
        for (int row = 0; row < feedbackTable_->rowCount(); ++row) {
            QString line;
            for (int col = 0; col < feedbackTable_->columnCount(); ++col) {
                line += feedbackTable_->item(row, col) ? feedbackTable_->item(row, col)->text() : "";
                if (col + 1 < feedbackTable_->columnCount()) {
                    line += ",";
                }
            }
            out << line << '\n';
        }

        QMessageBox::information(this, "Saved", "Feedback log saved successfully.");
    }

    void onFrameRefreshTick() {
        updateFramePreview();
    }

    void onFeedbackRefreshTick() {
        updateFeedbackTable();
    }

    void onUdpReadyRead() {
        while (udpSocket_->hasPendingDatagrams()) {
            QByteArray datagram;
            datagram.resize(static_cast<int>(udpSocket_->pendingDatagramSize()));
            udpSocket_->readDatagram(datagram.data(), datagram.size());
            processPackedFrames(datagram.constData(), datagram.size());
        }
    }

    void onTcpReadyRead() {
        tcpReceiveBuffer_.append(tcpSocket_->readAll());
        processPackedFrames(tcpReceiveBuffer_.constData(), tcpReceiveBuffer_.size(), true);
    }

    void onCanReadyRead() {
#ifdef Q_OS_LINUX
        while (true) {
            struct can_frame frame{};
            const ssize_t bytesRead =
                ::recv(canSocket_, &frame, sizeof(frame), MSG_DONTWAIT);
            if (bytesRead < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    break;
                }
                setConnectionStatus(false, "CAN RX ERROR");
                break;
            }
            if (bytesRead != static_cast<ssize_t>(sizeof(frame))) {
                break;
            }
            if ((frame.can_id & (CAN_RTR_FLAG | CAN_ERR_FLAG)) != 0u) {
                continue;
            }

            MotorPackMsg packet{};
            packet.id = (frame.can_id & CAN_EFF_FLAG) != 0u
                            ? (frame.can_id & CAN_EFF_MASK)
                            : (frame.can_id & CAN_SFF_MASK);
            packet.frame_flags =
                (frame.can_id & CAN_EFF_FLAG) != 0u ? MOTOR_CAN_FLAG_EFF : 0;
            packet.len = frame.can_dlc > 8 ? 8 : frame.can_dlc;
            std::memcpy(packet.data, frame.data, packet.len);
            updateFeedbackFromPacket(packet);
        }
#endif
    }

private:
    void initializeMotors() {
        motorList_.clear();
        for (int i = 0; i < 8; ++i) {
            MotorEntry entry;
            entry.id = i + 1;
            entry.name = QString("Motor %1").arg(i + 1);
            entry.enabled = true;
            entry.model = MOTOR_MODEL_EC_A2806_P2;
            entry.commandName = ((entry.id % 2) != 0) ? "spd" : "pos";
            entry.param1 = 0.0;
            entry.param2 = 1.5;
            entry.param3 = ((entry.id % 2) != 0) ? kFeedbackType1 : 0.0;
            entry.param4 = ((entry.id % 2) == 0) ? kFeedbackType1 : 0.0;
            motorList_.append(entry);
        }
    }

    void refreshHexByteEditorsForRow(int row) {
        if (!motorMatrix_ || row < 0 || row >= motorMatrix_->rowCount()) {
            return;
        }

        const int payloadLen = motorList_[row].rawDataLen;
        for (int byteIndex = 0; byteIndex < 8; ++byteIndex) {
            const int col = 4 + byteIndex;
            const bool visible = (byteIndex < payloadLen);
            motorMatrix_->setColumnHidden(col, !visible);

            auto* item = motorMatrix_->item(row, col);
            if (!item) {
                item = new QTableWidgetItem();
                item->setTextAlignment(Qt::AlignCenter);
                motorMatrix_->setItem(row, col, item);
            }

            if (visible) {
                item->setText(formatHexByte(motorList_[row].rawData[byteIndex]));
                item->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEnabled | Qt::ItemIsEditable);
            } else {
                item->setText("00");
                item->setFlags(Qt::NoItemFlags);
            }
        }
    }

    void buildMotorControlMatrix() {
        if (!motorMatrix_) {
            return;
        }

        const QStringList headers = {"Select", "Motor", "CAN ID", "Command", "D1", "D2", "D3", "D4",
                         "D5", "D6", "D7", "D8"};
        motorMatrix_->setRowCount(8);
        motorMatrix_->setColumnCount(static_cast<int>(headers.size()));
        motorMatrix_->setHorizontalHeaderLabels(headers);
        motorMatrix_->setAlternatingRowColors(true);
        motorMatrix_->setMinimumHeight(260);
        motorMatrix_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
        motorMatrix_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
        motorMatrix_->verticalHeader()->setVisible(false);

        connect(motorMatrix_, &QTableWidget::itemChanged, this, [this](QTableWidgetItem* item) {
            if (!item || !frameText_) {
                return;
            }
            const int col = item->column();
            if (col < 4 || col > 11) {
                return;
            }
            const int row = item->row();
            if (row < 0 || row >= motorList_.size()) {
                return;
            }

            const int byteIndex = col - 4;
            const QString text = item->text().trimmed().toUpper();
            if (text.isEmpty()) {
                item->setText("00");
                return;
            }

            bool ok = false;
            const uint value = text.toUInt(&ok, 16);
            if (!ok || text.size() > 2) {
                item->setText(formatHexByte(motorList_[row].rawData[byteIndex]));
                return;
            }

            motorList_[row].rawData[byteIndex] = static_cast<uint8_t>(value);
            motorList_[row].rawPayloadManual = true;
            item->setText(formatHexByte(motorList_[row].rawData[byteIndex]));
            updateFramePreview();
        });

        for (int i = 0; i < 8; ++i) {
            auto* enableBox = new QCheckBox(this);
            enableBox->setChecked(motorList_[i].enabled);
            enableBox->setToolTip(QString("Enable motor %1").arg(i + 1));
            connect(enableBox, &QCheckBox::toggled, this, [this, i](bool checked) {
                motorList_[i].enabled = checked;
                updateFramePreview();
            });
            motorMatrix_->setCellWidget(i, 0, enableBox);

            auto* motorLabel = new QLabel(QString("M%1").arg(i + 1), this);
            motorLabel->setAlignment(Qt::AlignCenter);
            motorMatrix_->setCellWidget(i, 1, motorLabel);

            auto* idLabel = new QLabel(QString::number(motorList_[i].id), this);
            idLabel->setAlignment(Qt::AlignCenter);
            motorMatrix_->setCellWidget(i, 2, idLabel);

            auto* cmd = new QComboBox(this);
            cmd->addItems({"spd", "pos", "cur", "tor", "stop", "set-pos", "set-id"});
            cmd->setCurrentText(motorList_[i].commandName);
            connect(cmd, QOverload<const QString&>::of(&QComboBox::currentIndexChanged), this,
                    [this, i](const QString& name) {
                        motorList_[i].commandName = name;
                        applyDefaultRawData(motorList_[i]);
                        refreshHexByteEditorsForRow(i);
                        updateFramePreview();
                    });
            motorMatrix_->setCellWidget(i, 3, cmd);

            for (int byteIndex = 0; byteIndex < 8; ++byteIndex) {
                auto* item = new QTableWidgetItem();
                item->setText(formatHexByte(motorList_[i].rawData[byteIndex]));
                item->setTextAlignment(Qt::AlignCenter);
                item->setFlags(item->flags() | Qt::ItemIsEditable);
                motorMatrix_->setItem(i, 4 + byteIndex, item);
            }

            applyDefaultRawData(motorList_[i]);
            refreshHexByteEditorsForRow(i);
        }
    }

    void buildUi() {
        auto* central = new QWidget(this);
        setCentralWidget(central);

        auto* rootLayout = new QHBoxLayout(central);

        auto* leftPanel = new QWidget(this);
        leftPanel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
        auto* leftLayout = new QVBoxLayout(leftPanel);

        auto* transportGroup = new QGroupBox("Transport Selection", this);
        auto* transportLayout = new QFormLayout(transportGroup);
        transportCombo_ = new QComboBox(this);
        transportCombo_->addItem("UDP-CAN", static_cast<int>(TransportMode::UdpCan));
        transportCombo_->addItem("TCP-TCP", static_cast<int>(TransportMode::TcpTcp));
#ifdef Q_OS_LINUX
        transportCombo_->addItem("SocketCAN", static_cast<int>(TransportMode::CanSocketCan));
#endif
        transportLayout->addRow("Path:", transportCombo_);

        hostEdit_ = new QLineEdit("192.168.1.50", this);
        portEdit_ = new QSpinBox(this);
        portEdit_->setRange(1, 65535);
        portEdit_->setValue(5000);
        transportLayout->addRow(QString("Target IP:"), hostEdit_);
        transportLayout->addRow(QString("Port:"), portEdit_);

        canInterfaceEdit_ = new QLineEdit("can0", this);
        transportLayout->addRow(QString("CAN iface:"), canInterfaceEdit_);

        canBitrateEdit_ = new QLineEdit("500000", this);
        canBitrateEdit_->setPlaceholderText("500000");
        canBitrateSetButton_ = new QPushButton("Set", this);
        auto* canBitrateRow = new QWidget(this);
        auto* canBitrateLayout = new QHBoxLayout(canBitrateRow);
        canBitrateLayout->setContentsMargins(0, 0, 0, 0);
        canBitrateLayout->addWidget(canBitrateEdit_);
        canBitrateLayout->addWidget(canBitrateSetButton_);
        transportLayout->addRow(QString("CAN bitrate:"), canBitrateRow);
        connect(canBitrateSetButton_, &QPushButton::clicked, this, &MotorControlWindow::onCanBitrateSetClicked);

        localIpLabel_ = new QLabel("Local IP: unknown", this);
        connectionStatusLabel_ = new QLabel("DISCONNECTED", this);
        connectionStatusLabel_->setAlignment(Qt::AlignCenter);
        connectionStatusLabel_->setFixedWidth(120);
        setConnectionStatus(false, "DISCONNECTED");
        transportLayout->addRow("Local IP:", localIpLabel_);
        transportLayout->addRow("Status:", connectionStatusLabel_);

        networkStatusLabel_ = new QLabel("NET: OFF", this);
        networkStatusLabel_->setAlignment(Qt::AlignCenter);
        networkStatusLabel_->setFixedWidth(120);
        setNetworkStatus(false, "NET: OFF");
        transportLayout->addRow("Net status:", networkStatusLabel_);

        connectButton_ = new QPushButton("Connect Network", this);
        connectButton_->setCheckable(true);
        transportLayout->addRow("Net:", connectButton_);
        connect(connectButton_, &QPushButton::clicked, this, &MotorControlWindow::onNetworkToggleClicked);

        auto* fullScreenButton = new QPushButton("Full Screen", this);
        transportLayout->addRow("View:", fullScreenButton);
        connect(fullScreenButton, &QPushButton::clicked, this, [this, fullScreenButton]() {
            if (isFullScreen()) {
                showNormal();
                fullScreenButton->setText("Full Screen");
            } else {
                showFullScreen();
                fullScreenButton->setText("Exit Full Screen");
            }
        });

        sendRateLabel_ = new QLabel("Send rate: 10 ms (100 Hz)", this);
        sendRateEdit_ = new QSpinBox(this);
        sendRateEdit_->setRange(10, 1000);
        sendRateEdit_->setSingleStep(10);
        sendRateEdit_->setValue(10);
        sendRateSetButton_ = new QPushButton("Set", this);
        auto* sendRateRow = new QWidget(this);
        auto* sendRateRowLayout = new QHBoxLayout(sendRateRow);
        sendRateRowLayout->setContentsMargins(0, 0, 0, 0);
        sendRateRowLayout->addWidget(sendRateEdit_);
        sendRateRowLayout->addWidget(sendRateSetButton_);
        transportLayout->addRow(sendRateLabel_, sendRateRow);
        connect(sendRateEdit_, QOverload<int>::of(&QSpinBox::valueChanged), this, &MotorControlWindow::onSendRateChanged);
        connect(sendRateSetButton_, &QPushButton::clicked, this, &MotorControlWindow::onSendRateSetClicked);

        auto* commandGroup = new QGroupBox("Per-Motor Command Matrix", this);
        auto* commandLayout = new QVBoxLayout(commandGroup);
        motorMatrix_ = new QTableWidget(this);
        buildMotorControlMatrix();
        commandLayout->addWidget(motorMatrix_);

        auto* matrixButtons = new QWidget(this);
        auto* matrixButtonLayout = new QHBoxLayout(matrixButtons);
        auto* selectAllButton = new QPushButton("Select All", this);
        auto* clearAllButton = new QPushButton("Clear All", this);
        matrixButtonLayout->addWidget(selectAllButton);
        matrixButtonLayout->addWidget(clearAllButton);
        connect(selectAllButton, &QPushButton::clicked, this, &MotorControlWindow::onSelectAllClicked);
        connect(clearAllButton, &QPushButton::clicked, this, &MotorControlWindow::onClearAllClicked);
        commandLayout->addWidget(matrixButtons);

        auto* directionControls = new QWidget(this);
        auto* directionLayout = new QHBoxLayout(directionControls);
        auto* positionGrid = new QGridLayout();
        const auto createPositionEditor = [this](int canId, QDoubleSpinBox*& editor) {
            auto* field = new QWidget(this);
            auto* fieldLayout = new QHBoxLayout(field);
            fieldLayout->setContentsMargins(0, 0, 0, 0);
            fieldLayout->addWidget(new QLabel(QString("CAN %1 POS:").arg(canId), field));
            editor = new QDoubleSpinBox(field);
            editor->setRange(-100000.0, 100000.0);
            editor->setDecimals(3);
            editor->setSingleStep(0.1);
            fieldLayout->addWidget(editor);
            return field;
        };
        positionGrid->addWidget(createPositionEditor(8, positionCan8Edit_), 0, 0);
        positionGrid->addWidget(createPositionEditor(2, positionCan2Edit_), 0, 1);
        positionGrid->addWidget(createPositionEditor(6, positionCan6Edit_), 1, 0);
        positionGrid->addWidget(createPositionEditor(4, positionCan4Edit_), 1, 1);
        directionLayout->addLayout(positionGrid, 1);

        auto* setDirectionButton = new QPushButton("Set Direction", this);
        directionLayout->addWidget(setDirectionButton);
        connect(setDirectionButton, &QPushButton::clicked, this, &MotorControlWindow::onSetDirectionClicked);
        commandLayout->addWidget(directionControls);

        auto* speedControls = new QWidget(this);
        auto* speedLayout = new QVBoxLayout(speedControls);
        auto* speedRowLayout = new QHBoxLayout();
        auto* speedGrid = new QGridLayout();
        const auto createSpeedEditor = [this](int canId, QDoubleSpinBox*& editor) {
            auto* field = new QWidget(this);
            auto* fieldLayout = new QHBoxLayout(field);
            fieldLayout->setContentsMargins(0, 0, 0, 0);
            fieldLayout->addWidget(new QLabel(QString("CAN %1 speed:").arg(canId), field));
            editor = new QDoubleSpinBox(field);
            editor->setRange(-100000.0, 100000.0);
            editor->setDecimals(3);
            editor->setSingleStep(0.1);
            fieldLayout->addWidget(editor);
            return field;
        };
        speedGrid->addWidget(createSpeedEditor(7, speedCan7Edit_), 0, 0);
        speedGrid->addWidget(createSpeedEditor(1, speedCan1Edit_), 0, 1);
        speedGrid->addWidget(createSpeedEditor(5, speedCan5Edit_), 1, 0);
        speedGrid->addWidget(createSpeedEditor(3, speedCan3Edit_), 1, 1);
        speedRowLayout->addLayout(speedGrid, 1);

        auto* setSpeedButton = new QPushButton("Set Speed", this);
        speedRowLayout->addWidget(setSpeedButton);
        connect(setSpeedButton, &QPushButton::clicked, this, &MotorControlWindow::onSetSpeedClicked);
        speedLayout->addLayout(speedRowLayout);

        auto* calibrationDirectionLayout = new QHBoxLayout();

        calibrationButton_ = new QToolButton(speedControls);
        calibrationButton_->setText("Calibration");
        calibrationButton_->setFixedSize(86, 38);
        calibrationButton_->setToolTip(
            "Read current speed from CAN 1/3/5/7 and position from CAN 2/4/6/8");
        calibrationDirectionLayout->addWidget(calibrationButton_, 1, Qt::AlignCenter);
        connect(calibrationButton_, &QToolButton::clicked, this,
                &MotorControlWindow::onCalibrationClicked);

        auto* dpad = new QWidget(this);
        auto* dpadLayout = new QGridLayout(dpad);
        dpadLayout->setContentsMargins(4, 0, 4, 0);
        dpadLayout->setSpacing(2);
        const auto createArrowButton = [dpad](Qt::ArrowType direction, const QString& name) {
            auto* button = new QToolButton(dpad);
            button->setArrowType(direction);
            button->setFixedSize(38, 38);
            button->setToolTip(name);
            button->setAccessibleName(name);
            return button;
        };
        auto* arrowUpButton = createArrowButton(Qt::UpArrow, "Up");
        auto* arrowDownButton = createArrowButton(Qt::DownArrow, "Down");
        auto* arrowLeftButton = createArrowButton(Qt::LeftArrow, "Left");
        auto* arrowRightButton = createArrowButton(Qt::RightArrow, "Right");
        dpadLayout->addWidget(arrowUpButton, 0, 1);
        dpadLayout->addWidget(arrowLeftButton, 1, 0);
        dpadLayout->addWidget(arrowRightButton, 1, 2);
        dpadLayout->addWidget(arrowDownButton, 2, 1);
        calibrationDirectionLayout->addWidget(dpad, 1, Qt::AlignCenter);
        auto* emergencyStopButton = new QPushButton("Emergency Stop", speedControls);
        emergencyStopButton->setMinimumHeight(38);
        emergencyStopButton->setToolTip("Set all eight motors to the stop command");
        emergencyStopButton->setStyleSheet(
            "QPushButton { background-color: #d32f2f; color: white; font-weight: bold; "
            "border: 1px solid #a80000; border-radius: 4px; padding: 6px 12px; }"
            "QPushButton:hover { background-color: #e53935; }"
            "QPushButton:pressed { background-color: #b71c1c; }");
        calibrationDirectionLayout->addWidget(emergencyStopButton, 1, Qt::AlignCenter);
        speedLayout->addLayout(calibrationDirectionLayout);
        connect(arrowUpButton, &QToolButton::clicked, this, &MotorControlWindow::onArrowUpClicked);
        connect(arrowDownButton, &QToolButton::clicked, this, &MotorControlWindow::onArrowDownClicked);
        connect(arrowLeftButton, &QToolButton::clicked, this, &MotorControlWindow::onArrowLeftClicked);
        connect(arrowRightButton, &QToolButton::clicked, this, &MotorControlWindow::onArrowRightClicked);
        connect(emergencyStopButton, &QPushButton::clicked, this,
                &MotorControlWindow::onStopClicked);
        commandLayout->addWidget(speedControls);

        auto* sendControls = new QWidget(this);
        auto* sendControlsLayout = new QHBoxLayout(sendControls);
        sendControlsLayout->setContentsMargins(0, 0, 0, 0);
        sendButton_ = new QPushButton("Send Enabled Motors", sendControls);
        auto* stopSendingButton = new QPushButton("Stop Sending", sendControls);
        stopSendingButton->setToolTip(
            "Stop periodic motor data transmission without disconnecting the network");
        sendControlsLayout->addWidget(sendButton_);
        sendControlsLayout->addWidget(stopSendingButton);
        saveButton_ = new QPushButton("Save feedback log", this);
        leftLayout->addWidget(transportGroup);
        leftLayout->addWidget(commandGroup);
        leftLayout->addWidget(sendControls);
        leftLayout->addWidget(saveButton_);
        leftLayout->addStretch();

        connect(sendButton_, &QPushButton::clicked, this, &MotorControlWindow::onSendClicked);
        connect(stopSendingButton, &QPushButton::clicked, this,
                &MotorControlWindow::onStopSendingClicked);
        connect(saveButton_, &QPushButton::clicked, this, &MotorControlWindow::onSaveFeedbackClicked);

        rightPanel_ = new QWidget(this);
        rightPanel_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        auto* rightLayout = new QVBoxLayout(rightPanel_);

        auto* frameGroup = new QGroupBox("Command Frame Preview (0.5s refresh)", this);
        auto* frameLayout = new QVBoxLayout(frameGroup);
        frameText_ = new QTextEdit(this);
        frameText_->setReadOnly(true);
        frameText_->setFont(QFontDatabase::systemFont(QFontDatabase::FixedFont));
        frameLayout->addWidget(frameText_);

        auto* feedbackGroup = new QGroupBox("8 Motor Feedback Data (1s refresh)", this);
        auto* feedbackLayout = new QVBoxLayout(feedbackGroup);
        feedbackTable_ = new QTableWidget(8, 9, this);
        feedbackTable_->setHorizontalHeaderLabels(
            {"Motor", "CAN ACK (hex)", "Status", "Position (rad)", "Speed (rad/s)",
             "Current (A)", "Motor (C)", "MOS (C)", "Last RX"});
        feedbackTable_->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
        feedbackTable_->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
        feedbackTable_->verticalHeader()->setVisible(false);
        feedbackLayout->addWidget(feedbackTable_);

        rightLayout->addWidget(frameGroup, 1);
        rightLayout->addWidget(feedbackGroup, 1);

        auto* leftScrollArea = new QScrollArea(central);
        leftScrollArea->setWidgetResizable(true);
        leftScrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
        leftScrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
        leftScrollArea->setFrameShape(QFrame::NoFrame);
        leftScrollArea->setWidget(leftPanel);

        rootLayout->addWidget(leftScrollArea, 1);
        rootLayout->addWidget(rightPanel_, 1);
    }

    void updateFramePreview() {
        if (!frameText_) {
            return;
        }

        QByteArray currentFrame;

        for (const auto& motor : motorList_) {
            if (!motor.enabled) {
                continue;
            }
            const QByteArray packetBytes = buildMotorPacketBytes(
                motor, motor.commandName, motor.param1, motor.param2, motor.param3, motor.param4);
            currentFrame.append(packetBytes);
        }

        frameText_->clear();
        frameText_->append(QString("Transport: %1\n").arg(
            transportCombo_->currentText()));
        frameText_->append("Selected motors:\n");
        for (const auto& motor : motorList_) {
            if (motor.enabled) {
                frameText_->append(QString("  M%1 -> %2 p1=%3 p2=%4 p3=%5 p4=%6\n")
                                      .arg(motor.id)
                                      .arg(motor.commandName)
                                      .arg(motor.param1)
                                      .arg(motor.param2)
                                      .arg(motor.param3)
                                      .arg(motor.param4));
            }
        }
        frameText_->append("\n");

        if (currentFrame.isEmpty()) {
            frameText_->append("No enabled motors.");
        } else {
            const int bytesPerFrame = static_cast<int>(sizeof(MotorPackMsg));
            for (int i = 0; i < currentFrame.size(); i += bytesPerFrame) {
                const QByteArray slice = currentFrame.mid(i, bytesPerFrame);
                frameText_->append(packetToText(slice));
            }
        }
    }

    void updateFeedbackTable() {
        feedbackTable_->setRowCount(8);

        const QDateTime now = QDateTime::currentDateTime();

        for (int i = 0; i < 8; ++i) {
            QStringList values = {QString("M%1").arg(i + 1),
                                  QString("0x%1 [--]").arg(i + 1, 3, 16, QChar('0')).toUpper(),
                                  "WAITING", "-", "-", "-", "-", "-", "-"};
            const MotorFeedbackState& state = feedbackStates_[static_cast<size_t>(i)];
            if (state.received) {
                const qint64 ageMs = state.receivedAt.msecsTo(now);
                QString status;
                if (state.decoded.error != 0) {
                    status = QString("ERR %1").arg(state.decoded.error);
                } else if (ageMs > 2000) {
                    status = QString("STALE %1s").arg(static_cast<double>(ageMs) / 1000.0, 0,
                                                       'f', 1);
                } else {
                    status = "OK";
                }

                values = QStringList{QString("M%1").arg(i + 1),
                                     feedbackFrameToText(state.packet),
                                     status,
                                     QString::number(state.decoded.position_rad, 'f', 4),
                                     QString::number(state.decoded.speed_rad_s, 'f', 4),
                                     QString::number(state.decoded.current_a, 'f', 4),
                                     QString::number(state.decoded.motor_temperature_c, 'f', 1),
                                     QString::number(state.decoded.mos_temperature_c, 'f', 1),
                                     state.receivedAt.toString("hh:mm:ss.zzz")};
            }

            for (int column = 0; column < values.size(); ++column) {
                if (!feedbackTable_->item(i, column)) {
                    feedbackTable_->setItem(i, column, new QTableWidgetItem());
                }
                feedbackTable_->item(i, column)->setText(values[column]);
            }
        }
    }

    void processPackedFrames(const char* bytes, int size, bool keepRemainder = false) {
        const int frameSize = static_cast<int>(sizeof(MotorPackMsg));
        int offset = 0;
        while (offset + frameSize <= size) {
            MotorPackMsg packet{};
            std::memcpy(&packet, bytes + offset, frameSize);
            if (packet.len <= sizeof(packet.data)) {
                updateFeedbackFromPacket(packet);
            }
            offset += frameSize;
        }

        if (keepRemainder) {
            tcpReceiveBuffer_.remove(0, offset);
        }
    }

    void updateFeedbackFromPacket(const MotorPackMsg& packet) {
        if (processCalibrationResponse(packet)) {
            return;
        }
        if (packet.id < 1 || packet.id > static_cast<uint32_t>(motorList_.size()) || packet.len < 8) {
            return;
        }

        const int row = static_cast<int>(packet.id) - 1;
        MotorRanges ranges{};
        const float currentRange = motor_get_model_ranges(motorList_[row].model, &ranges)
                                       ? ranges.current.max
                                       : 40.0f;
        MotorFeedback1 decoded{};
        if (!motor_decode_feedback1(&packet, currentRange, &decoded)) {
            return;
        }

        MotorFeedbackState& state = feedbackStates_[static_cast<size_t>(row)];
        state.packet = packet;
        state.decoded = decoded;
        state.receivedAt = QDateTime::currentDateTime();
        state.received = true;
    }

    QDoubleSpinBox* calibrationEditor(int motorId) const {
        switch (motorId) {
            case 1:
                return speedCan1Edit_;
            case 2:
                return positionCan2Edit_;
            case 3:
                return speedCan3Edit_;
            case 4:
                return positionCan4Edit_;
            case 5:
                return speedCan5Edit_;
            case 6:
                return positionCan6Edit_;
            case 7:
                return speedCan7Edit_;
            case 8:
                return positionCan8Edit_;
            default:
                return nullptr;
        }
    }

    bool processCalibrationResponse(const MotorPackMsg& packet) {
        if (packet.len < 2 || (packet.data[0] >> 5) != 5) {
            return false;
        }
        if (!calibrationActive_ || packet.id < 1 || packet.id > 8) {
            return true;
        }

        const uint8_t responseError = static_cast<uint8_t>(packet.data[0] & 0x1F);
        if (responseError != 0) {
            finishCalibration(false,
                              QString("CAN %1 rejected the query (error %2).")
                                  .arg(packet.id)
                                  .arg(responseError));
            return true;
        }

        MotorQueryFeedbackFloat feedback{};
        if (!motor_decode_query_feedback_float(&packet, &feedback)) {
            return true;
        }

        const int motorId = static_cast<int>(packet.id);
        const uint8_t expectedCode = ((motorId % 2) != 0)
                                         ? static_cast<uint8_t>(MOTOR_PARAM_SPEED)
                                         : static_cast<uint8_t>(MOTOR_PARAM_POSITION);
        if (feedback.query_code != expectedCode ||
            !calibrationPending_[static_cast<size_t>(motorId - 1)]) {
            return true;
        }
        if (feedback.error != 0 || !std::isfinite(feedback.value)) {
            finishCalibration(false,
                              QString("CAN %1 returned an invalid query response (error %2).")
                                  .arg(motorId)
                                  .arg(feedback.error));
            return true;
        }

        QDoubleSpinBox* editor = calibrationEditor(motorId);
        if (!editor) {
            return true;
        }
        editor->setValue(feedback.value);
        calibrationPending_[static_cast<size_t>(motorId - 1)] = false;

        if (std::none_of(calibrationPending_.begin(), calibrationPending_.end(),
                         [](bool pending) { return pending; })) {
            onSetSpeedClicked();
            onSetDirectionClicked();
            finishCalibration(true, "All eight motor values were calibrated.");
        }
        return true;
    }

    void finishCalibration(bool success, const QString& message) {
        if (!calibrationActive_) {
            return;
        }

        calibrationTimer_->stop();
        calibrationActive_ = false;
        calibrationButton_->setEnabled(true);
        calibrationButton_->setText("Calibration");
        if (calibrationResumeAutoSend_ && networkConnected_) {
            autoSendTimer_->start(currentSendIntervalMs());
        }
        calibrationResumeAutoSend_ = false;

        if (success) {
            QMessageBox::information(this, "Calibration", message);
        } else {
            QMessageBox::warning(this, "Calibration", message);
        }
    }

    void refreshLocalAddressInfo() {
        QStringList ips;
        for (const QNetworkInterface& iface : QNetworkInterface::allInterfaces()) {
            if (((iface.flags() & QNetworkInterface::IsUp) == 0) ||
                ((iface.flags() & QNetworkInterface::IsLoopBack) != 0)) {
                continue;
            }
            for (const QNetworkAddressEntry& entry : iface.addressEntries()) {
                const QHostAddress addr = entry.ip();
                if (addr.protocol() == QAbstractSocket::IPv4Protocol && !addr.isNull()) {
                    ips << addr.toString();
                }
            }
        }
        if (ips.isEmpty()) {
            localIpLabel_->setText("Local IP: none");
        } else {
            localIpLabel_->setText(QString("Local IP: %1").arg(ips.first()));
        }
    }

    void setConnectionStatus(bool connected, const QString& text) {
        const QString style = connected
            ? "QLabel { background-color: #2ecc71; color: white; border-radius: 8px; padding: 3px 8px; }"
            : "QLabel { background-color: #e74c3c; color: white; border-radius: 8px; padding: 3px 8px; }";
        connectionStatusLabel_->setText(text);
        connectionStatusLabel_->setStyleSheet(style);
    }

    bool sendCanFrame(const MotorPackMsg& packet, int& sentCount) {
#ifdef Q_OS_LINUX
        if (!ensureCanSocketOpen()) {
            return false;
        }

        struct can_frame frame{};
        frame.can_id = packet.id;
        if ((packet.frame_flags & MOTOR_CAN_FLAG_EFF) != 0u) {
            frame.can_id |= CAN_EFF_FLAG;
        }
        if ((packet.frame_flags & MOTOR_CAN_FLAG_RTR) != 0u) {
            frame.can_id |= CAN_RTR_FLAG;
        }
        frame.can_dlc = static_cast<__u8>(packet.len);
        std::memset(frame.data, 0, sizeof(frame.data));
        for (uint8_t i = 0; i < packet.len; ++i) {
            frame.data[i] = packet.data[i];
        }

        const ssize_t written = ::write(canSocket_, &frame, sizeof(frame));
        if (written <= 0) {
            return false;
        }
        sentCount += 1;
        return true;
#else
        Q_UNUSED(packet);
        Q_UNUSED(sentCount);
        return false;
#endif
    }

    void sendCurrentFrame() {
        if (calibrationActive_) {
            return;
        }

        QByteArray payload;
        QList<int> successfulRows;

        for (int i = 0; i < motorList_.size(); ++i) {
            const auto& motor = motorList_[i];
            if (!motor.enabled) {
                applyMotorRowVisualState(i, "idle");
                continue;
            }
            const QByteArray packetBytes = buildMotorPacketBytes(
                motor, motor.commandName, motor.param1, motor.param2, motor.param3, motor.param4);
            payload.append(packetBytes);
            successfulRows << i;
        }

        if (payload.isEmpty()) {
            QMessageBox::warning(this, "No motor selected", "Please enable at least one motor in the selection matrix.");
            return;
        }

        const int transport = transportCombo_->currentData().toInt();
        const bool udpMode = (transport == static_cast<int>(TransportMode::UdpCan));
        const bool tcpMode = (transport == static_cast<int>(TransportMode::TcpTcp));
        const bool canMode = (transport == static_cast<int>(TransportMode::CanSocketCan));
        bool sendOk = false;

        if (udpMode) {
            const qint64 sent = udpSocket_->writeDatagram(payload, QHostAddress(hostEdit_->text()),
                                                         portEdit_->value());
            sendOk = (sent == payload.size());
            if (sendOk) {
                setConnectionStatus(true, "CONNECTED");
                for (int row : successfulRows) {
                    applyMotorRowVisualState(row, "success");
                }
            } else {
                setConnectionStatus(false, "FAILED");
                for (int row : successfulRows) {
                    applyMotorRowVisualState(row, "fail");
                }
                QMessageBox::warning(this, "UDP send failed", QString("Failed to send %1 bytes to %2:%3").arg(payload.size()).arg(hostEdit_->text()).arg(portEdit_->value()));
            }
        } else if (tcpMode) {
            if (tcpSocket_->state() != QAbstractSocket::ConnectedState) {
                tcpSocket_->connectToHost(hostEdit_->text(), portEdit_->value());
            }
            if (tcpSocket_->state() == QAbstractSocket::ConnectedState || tcpSocket_->waitForConnected(1000)) {
                const qint64 queued = tcpSocket_->write(payload);
                tcpSocket_->waitForBytesWritten(1000);
                if (queued == payload.size()) {
                sendOk = true;
                setConnectionStatus(true, "CONNECTED");
                for (int row : successfulRows) {
                    applyMotorRowVisualState(row, "success");
                }
                }
            } else {
                setConnectionStatus(false, "FAILED");
                for (int row : successfulRows) {
                    applyMotorRowVisualState(row, "fail");
                }
                QMessageBox::warning(this, "TCP connect failed", "Unable to connect to TCP target");
            }
        } else if (canMode) {
            int sentFrames = 0;
            bool ok = true;
            for (int row : successfulRows) {
                const auto& motor = motorList_[row];
                MotorPackMsg packet{};
                std::memcpy(&packet, buildMotorPacketBytes(motor, motor.commandName, motor.param1,
                                                         motor.param2, motor.param3, motor.param4).constData(),
                            sizeof(packet));
                if (!sendCanFrame(packet, sentFrames)) {
                    ok = false;
                    applyMotorRowVisualState(row, "fail");
                    setConnectionStatus(false, "FAILED");
                    QMessageBox::warning(this, "SocketCAN send failed",
                                         QString("Failed to send CAN frame on %1").arg(canInterfaceEdit_->text()));
                    break;
                }
                applyMotorRowVisualState(row, "success");
            }
            sendOk = ok && (sentFrames > 0);
            if (sendOk) {
                setConnectionStatus(true, "CONNECTED");
            }
        }

        if (!sendOk) {
            return;
        }

        if (!autoSendTimer_->isActive() && networkConnected_) {
            autoSendTimer_->start(currentSendIntervalMs());
        }
    }

    void setNetworkStatus(bool connected, const QString& text) {
        const QString green = "QLabel { background-color: #2ecc71; color: white; border-radius: 8px; padding: 3px 8px; }";
        const QString red = "QLabel { background-color: #e74c3c; color: white; border-radius: 8px; padding: 3px 8px; }";
        networkStatusLabel_->setText(text);
        networkStatusLabel_->setStyleSheet(connected ? green : red);
    }

    void applyMotorRowVisualState(int row, const QString& state) {
        if (!motorMatrix_ || row < 0 || row >= motorMatrix_->rowCount()) {
            return;
        }

        const QStringList palette = {"#dfeeff", "#e6ffe6", "#fff0d9", "#ffe7e7", "#f0e7ff", "#e0fff8", "#fff2d9", "#f7d8ff"};
        const QColor defaultColor = QColor("#f5f5f5");
        const QColor successColor = QColor(palette[row % palette.size()]);
        const QColor failColor = QColor("#ff7a7a");
        const QColor disabledColor = QColor("#e6e6e6");

        for (int col = 0; col < motorMatrix_->columnCount(); ++col) {
            auto* w = motorMatrix_->cellWidget(row, col);
            if (!w) {
                continue;
            }
            const bool enabled = motorList_[row].enabled;
            if (!enabled) {
                w->setStyleSheet(QString("background-color:%1; color:#666666; border:1px solid #d9d9d9;").arg(disabledColor.name()));
                continue;
            }
            if (state == "success") {
                w->setStyleSheet(QString("background-color:%1; color:black; border:1px solid #9ad7a0;").arg(successColor.name()));
            } else if (state == "fail") {
                w->setStyleSheet(QString("background-color:%1; color:white; border:1px solid #d65a5a;").arg(failColor.name()));
            } else {
                w->setStyleSheet(QString("background-color:%1; color:black; border:1px solid #dfe8f0;").arg(defaultColor.name()));
            }
        }
    }

    int currentSendIntervalMs() const {
        return sendRateEdit_ ? sendRateEdit_->value() : 10;
    }

    bool probeNetwork() {
        const int transport = transportCombo_->currentData().toInt();

        if (transport == static_cast<int>(TransportMode::UdpCan)) {
            const QHostAddress host(hostEdit_->text());
            if (host.isNull()) {
                return false;
            }
            QByteArray ping("NET_CHECK");
            const qint64 sent = udpSocket_->writeDatagram(ping, host, portEdit_->value());
            return sent == ping.size();
        }

        if (transport == static_cast<int>(TransportMode::CanSocketCan)) {
            return ensureCanSocketOpen();
        }

        if (tcpSocket_->state() == QAbstractSocket::ConnectedState) {
            return true;
        }

        const QHostAddress host(hostEdit_->text());
        if (host.isNull()) {
            return false;
        }

        if (tcpSocket_->state() == QAbstractSocket::ConnectedState) {
            return true;
        }
        tcpSocket_->connectToHost(host, portEdit_->value());
        return tcpSocket_->waitForConnected(1000);
    }

    bool ensureCanSocketOpen() {
#ifdef Q_OS_LINUX
        if (canSocket_ >= 0) {
            return true;
        }
        const QByteArray iface = canInterfaceEdit_->text().trimmed().toLocal8Bit();
        if (iface.isEmpty()) {
            return false;
        }
        const int fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (fd < 0) {
            return false;
        }
        struct ifreq ifr{};
        std::snprintf(ifr.ifr_name, IFNAMSIZ, "%s", iface.constData());
        if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
            ::close(fd);
            return false;
        }
        struct sockaddr_can addr{};
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (bind(fd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
            ::close(fd);
            return false;
        }
        canSocket_ = fd;
        canReadNotifier_ = new QSocketNotifier(canSocket_, QSocketNotifier::Read, this);
        connect(canReadNotifier_, SIGNAL(activated(int)), this, SLOT(onCanReadyRead()));
        return true;
#else
        return false;
#endif
    }

    void closeCanSocket() {
        if (canReadNotifier_) {
            canReadNotifier_->setEnabled(false);
            delete canReadNotifier_;
            canReadNotifier_ = nullptr;
        }
        if (canSocket_ >= 0) {
#ifdef Q_OS_LINUX
            ::close(canSocket_);
#endif
            canSocket_ = -1;
        }
    }

    bool applySocketCanBitrate() {
#ifdef Q_OS_LINUX
        const QString iface = canInterfaceEdit_->text().trimmed();
        const QString bitrateText = canBitrateEdit_->text().trimmed();
        if (iface.isEmpty() || bitrateText.isEmpty()) {
            return false;
        }

        if (iface.startsWith("vcan")) {
            const QNetworkInterface virtualCan = QNetworkInterface::interfaceFromName(iface);
            return virtualCan.isValid() &&
                   ((virtualCan.flags() & QNetworkInterface::IsUp) != 0);
        }

        bool ok = false;
        const int bitrate = bitrateText.toInt(&ok);
        if (!ok || bitrate <= 0) {
            return false;
        }

        QProcess proc;
        proc.start("ip", {"link", "set", iface, "down"});
        if (!proc.waitForFinished(2000) || proc.exitCode() != 0) {
            return false;
        }

        proc.start("ip", {"link", "set", iface, "up", "type", "can", "bitrate",
                           QString::number(bitrate)});
        if (!proc.waitForFinished(2000) || proc.exitCode() != 0) {
            return false;
        }

        return true;
#else
        return false;
#endif
    }

    void onCanBitrateSetClicked() {
        const bool ok = applySocketCanBitrate();
        if (ok) {
            QMessageBox::information(this, "SocketCAN bitrate",
                                     QString("Applied bitrate %1 to %2.")
                                         .arg(canBitrateEdit_->text().trimmed())
                                         .arg(canInterfaceEdit_->text().trimmed()));
        } else {
            QMessageBox::warning(this, "SocketCAN bitrate",
                                 QString("Failed to set bitrate on %1. Ensure the interface exists and the program has permission to run 'ip link set'.")
                                     .arg(canInterfaceEdit_->text().trimmed()));
        }
    }

    void onNetworkToggleClicked() {
        if (connectButton_->isChecked()) {
            if (transportCombo_->currentData().toInt() == static_cast<int>(TransportMode::CanSocketCan)) {
                if (!applySocketCanBitrate()) {
                    QMessageBox::warning(this, "SocketCAN bitrate",
                                         QString("Bitrate could not be applied to %1. Check interface name and privileges.")
                                             .arg(canInterfaceEdit_->text().trimmed()));
                }
            }

            if (probeNetwork()) {
                networkConnected_ = true;
                setNetworkStatus(true, "CONNECTED");
                connectButton_->setText("Disconnect Network");
            } else {
                networkConnected_ = false;
                setNetworkStatus(false, "FAILED");
                connectButton_->setChecked(false);
                connectButton_->setText("Connect Network");
                autoSendTimer_->stop();
            }
        } else {
            networkConnected_ = false;
            setNetworkStatus(false, "DISCONNECTED");
            connectButton_->setText("Connect Network");
            autoSendTimer_->stop();
            if (tcpSocket_->state() != QAbstractSocket::UnconnectedState) {
                tcpSocket_->disconnectFromHost();
            }
            closeCanSocket();
        }
    }

    void onSendRateChanged(int value) {
        const int ms = value;
        const int hz = (ms > 0) ? (1000 / ms) : 0;
        sendRateLabel_->setText(QString("Send rate: %1 ms (%2 Hz)").arg(ms).arg(hz));
    }

    void onSendRateSetClicked() {
        const int ms = sendRateEdit_->value();
        const int hz = (ms > 0) ? (1000 / ms) : 0;
        sendRateLabel_->setText(QString("Send rate: %1 ms (%2 Hz)").arg(ms).arg(hz));
        if (networkConnected_ && autoSendTimer_->isActive()) {
            autoSendTimer_->start(ms);
        }
    }

    void onAutoSendTick() {
        if (!networkConnected_ || !probeNetwork()) {
            networkConnected_ = false;
            setNetworkStatus(false, "FAILED");
            connectButton_->setChecked(false);
            connectButton_->setText("Connect Network");
            autoSendTimer_->stop();
            return;
        }
        sendCurrentFrame();
    }

    void onNetworkCheck() {
        if (!networkConnected_) {
            return;
        }
        if (!probeNetwork()) {
            networkConnected_ = false;
            setNetworkStatus(false, "FAILED");
            connectButton_->setChecked(false);
            connectButton_->setText("Connect Network");
            autoSendTimer_->stop();
        }
    }

    QList<MotorEntry> motorList_;
    std::array<MotorFeedbackState, 8> feedbackStates_{};
    QComboBox* transportCombo_ = nullptr;
    QLineEdit* hostEdit_ = nullptr;
    QSpinBox* portEdit_ = nullptr;
    QLineEdit* canInterfaceEdit_ = nullptr;
    QLineEdit* canBitrateEdit_ = nullptr;
    QPushButton* canBitrateSetButton_ = nullptr;
    QDoubleSpinBox* positionCan8Edit_ = nullptr;
    QDoubleSpinBox* positionCan2Edit_ = nullptr;
    QDoubleSpinBox* positionCan6Edit_ = nullptr;
    QDoubleSpinBox* positionCan4Edit_ = nullptr;
    QDoubleSpinBox* speedCan7Edit_ = nullptr;
    QDoubleSpinBox* speedCan1Edit_ = nullptr;
    QDoubleSpinBox* speedCan5Edit_ = nullptr;
    QDoubleSpinBox* speedCan3Edit_ = nullptr;
    QTableWidget* motorMatrix_ = nullptr;
    QPushButton* sendButton_ = nullptr;
    QPushButton* saveButton_ = nullptr;
    QPushButton* connectButton_ = nullptr;
    QTextEdit* frameText_ = nullptr;
    QTableWidget* feedbackTable_ = nullptr;
    QWidget* rightPanel_ = nullptr;
    QLabel* localIpLabel_ = nullptr;
    QLabel* connectionStatusLabel_ = nullptr;
    QLabel* sendRateLabel_ = nullptr;
    QLabel* networkStatusLabel_ = nullptr;
    QSpinBox* sendRateEdit_ = nullptr;
    QPushButton* sendRateSetButton_ = nullptr;
    QTimer* frameTimer_ = nullptr;
    QTimer* feedbackTimer_ = nullptr;
    QTimer* autoSendTimer_ = nullptr;
    QTimer* networkCheckTimer_ = nullptr;
    QTimer* calibrationTimer_ = nullptr;
    QUdpSocket* udpSocket_ = nullptr;
    QTcpSocket* tcpSocket_ = nullptr;
    QSocketNotifier* canReadNotifier_ = nullptr;
    QToolButton* calibrationButton_ = nullptr;
    QByteArray tcpReceiveBuffer_;
    std::array<bool, 8> calibrationPending_{};
    int canSocket_ = -1;
    bool networkConnected_ = false;
    bool calibrationActive_ = false;
    bool calibrationResumeAutoSend_ = false;
};

int main(int argc, char** argv) {
    QApplication app(argc, argv);
    MotorControlWindow window;
    window.show();
    return app.exec();
}

#include "qt_motor_control_ui.moc"

