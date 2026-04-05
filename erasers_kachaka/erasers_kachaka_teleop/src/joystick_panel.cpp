#include "erasers_kachaka_teleop/joystick_panel.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QPainter>
#include <QMouseEvent>
#include <QPolygon>
#include <cmath>
#include <algorithm>

namespace erasers_kachaka_teleop
{

// =========================================================================
// JoystickWidget Implementation
// =========================================================================
JoystickWidget::JoystickWidget(QWidget * parent) : QWidget(parent) {
  setMinimumSize(250, 250);
  
  // ★変更点1: StrongFocusに変更（ClickFocusより強力で、プログラムからのフォーカス移動を受け入れやすい）
  setFocusPolicy(Qt::StrongFocus);
}

float JoystickWidget::getLinear() const {
  return std::clamp(mouse_linear_ + key_linear_, -1.0f, 1.0f);
}

float JoystickWidget::getAngular() const {
  return std::clamp(mouse_angular_ + key_angular_, -1.0f, 1.0f);
}

bool JoystickWidget::isActive() const {
  return mouse_pressed_ || w_down_ || s_down_ || a_down_ || d_down_;
}

void JoystickWidget::resizeEvent(QResizeEvent * /*event*/) {
  center_ = QPoint(width() / 2, height() / 2);
  // 操作中でなければ中央に戻す
  if (!isActive()) {
    stick_pos_ = center_;
  }
}

void JoystickWidget::changeEvent(QEvent * event) {
  if (event->type() == QEvent::EnabledChange) {
    if (!isEnabled()) {
      // 無効化されたら全リセット
      w_down_ = s_down_ = a_down_ = d_down_ = false;
      key_linear_ = 0.0f;
      key_angular_ = 0.0f;
      mouse_pressed_ = false;
      stick_pos_ = center_;
    }
    update();
  }
  QWidget::changeEvent(event);
}

void JoystickWidget::focusInEvent(QFocusEvent * /*event*/) {
  update(); // 枠線を表示するために再描画
}

void JoystickWidget::focusOutEvent(QFocusEvent * /*event*/) {
  // フォーカスが外れたらキー入力をリセット（安全のため）
  w_down_ = s_down_ = a_down_ = d_down_ = false;
  key_linear_ = 0.0f;
  key_angular_ = 0.0f;
  
  // マウス操作中でなければ見た目も戻す
  if (!mouse_pressed_) {
    stick_pos_ = center_;
  }
  update();
}

void JoystickWidget::paintEvent(QPaintEvent * /*event*/) {
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);
  
  bool enabled = isEnabled();
  bool has_focus = hasFocus();

  QColor bg_color = enabled ? QColor(255, 255, 255) : QColor(100, 100, 100);
  QColor base_color = enabled ? QColor(220, 220, 220) : QColor(80, 80, 80);
  QColor stick_color = enabled ? QColor(60, 60, 60) : QColor(150, 150, 150, 150);
  QColor arrow_color = enabled ? QColor(150, 150, 150) : QColor(80, 80, 80);

  painter.fillRect(rect(), bg_color);

  // 青いフォーカス枠の描画
  if (enabled && has_focus) {
    QPen pen(QColor(0, 120, 255), 4); // 少し太くしました
    painter.setPen(pen);
    painter.setBrush(Qt::NoBrush);
    painter.drawRect(rect().adjusted(2, 2, -2, -2));
    
    // ガイドテキスト
    painter.setPen(QColor(0, 120, 255));
    painter.setFont(QFont("Arial", 10, QFont::Bold));
    painter.drawText(rect().adjusted(8, 8, -8, -8), Qt::AlignTop | Qt::AlignLeft, "WASD Active");
  }

  // 矢印
  painter.setPen(Qt::NoPen);
  painter.setBrush(arrow_color);
  int arrow_dist = joy_radius_ + 20; 
  int arrow_size = 15;
  for (int i = 0; i < 4; ++i) {
    painter.save();
    painter.translate(center_);
    painter.rotate(i * 90); 
    QPolygon arrow;
    arrow << QPoint(0, -arrow_dist - arrow_size)
          << QPoint(-arrow_size / 2, -arrow_dist)
          << QPoint(arrow_size / 2, -arrow_dist);
    painter.drawPolygon(arrow);
    painter.restore();
  }

  // ベース円
  painter.setBrush(base_color);
  painter.drawEllipse(center_, joy_radius_, joy_radius_);

  // スティック
  painter.setBrush(stick_color);
  painter.drawEllipse(stick_pos_, stick_radius_, stick_radius_);
}

void JoystickWidget::mousePressEvent(QMouseEvent * event) {
  if (!isEnabled()) return;
  
  setFocus(); // クリックでフォーカス取得

  QPoint diff = event->pos() - center_;
  if (std::sqrt(diff.x()*diff.x() + diff.y()*diff.y()) < joy_radius_) {
    mouse_pressed_ = true;
    updateStickPos(event->pos());
  }
}

void JoystickWidget::mouseMoveEvent(QMouseEvent * event) {
  if (!isEnabled()) return;
  if (mouse_pressed_) {
    updateStickPos(event->pos());
  }
}

void JoystickWidget::mouseReleaseEvent(QMouseEvent * /*event*/) {
  mouse_pressed_ = false;
  
  // キー入力がなければ中央に戻す
  if (key_linear_ == 0.0f && key_angular_ == 0.0f) {
    stick_pos_ = center_;
  } else {
    calcStickPosFromVelocity(); // キー入力があるならその位置へ
  }

  mouse_linear_ = 0.0f;
  mouse_angular_ = 0.0f;
  update();
}

void JoystickWidget::updateStickPos(const QPoint & pos) {
  QPoint diff = pos - center_;
  double dist = std::sqrt(diff.x()*diff.x() + diff.y()*diff.y());
  double angle = std::atan2(diff.y(), diff.x());
  if (dist > joy_radius_) {
    stick_pos_.setX(center_.x() + joy_radius_ * std::cos(angle));
    stick_pos_.setY(center_.y() + joy_radius_ * std::sin(angle));
  } else {
    stick_pos_ = pos;
  }
  
  mouse_linear_ = -((float)(stick_pos_.y() - center_.y()) / joy_radius_);
  mouse_angular_ = -((float)(stick_pos_.x() - center_.x()) / joy_radius_);
  update();
}

void JoystickWidget::keyPressEvent(QKeyEvent * event)
{
  if (!isEnabled()) return;

  switch (event->key()) {
    case Qt::Key_W: w_down_ = true; break;
    case Qt::Key_S: s_down_ = true; break;
    case Qt::Key_A: a_down_ = true; break;
    case Qt::Key_D: d_down_ = true; break;
    default: QWidget::keyPressEvent(event); return;
  }
  updateVelocity();
}

void JoystickWidget::keyReleaseEvent(QKeyEvent * event)
{
  switch (event->key()) {
    case Qt::Key_W: w_down_ = false; break;
    case Qt::Key_S: s_down_ = false; break;
    case Qt::Key_A: a_down_ = false; break;
    case Qt::Key_D: d_down_ = false; break;
    default: QWidget::keyReleaseEvent(event); return;
  }
  updateVelocity();
}

void JoystickWidget::updateVelocity()
{
  key_linear_ = 0.0f;
  key_angular_ = 0.0f;

  if (w_down_) key_linear_ += 1.0f;
  if (s_down_) key_linear_ -= 1.0f;
  if (a_down_) key_angular_ += 1.0f;
  if (d_down_) key_angular_ -= 1.0f;
  
  // マウス操作中でなければ、キー入力に合わせてスティックの絵を動かす
  if (!mouse_pressed_) {
    calcStickPosFromVelocity();
  }
  update(); 
}

// キー入力に応じてスティックを擬似的に動かす（視覚効果）
void JoystickWidget::calcStickPosFromVelocity()
{
  float target_x = -key_angular_ * joy_radius_; // angularはX軸反転
  float target_y = -key_linear_ * joy_radius_;  // linearはY軸反転

  stick_pos_.setX(center_.x() + target_x);
  stick_pos_.setY(center_.y() + target_y);
}


// =========================================================================
// JoystickPanel Implementation
// =========================================================================

JoystickPanel::JoystickPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  auto options = rclcpp::NodeOptions().arguments({});
  node_ = std::make_shared<rclcpp::Node>("rviz_joystick_panel_node", options);

  QVBoxLayout * main_layout = new QVBoxLayout;
  tab_widget_ = new QTabWidget;
  main_layout->addWidget(tab_widget_);
  setLayout(main_layout);

  initControlTab();
  initSettingsTab();

  output_timer_ = new QTimer(this);
  connect(output_timer_, SIGNAL(timeout()), this, SLOT(sendVel()));
  output_timer_->start(100);

  topic_timer_ = new QTimer(this);
  connect(topic_timer_, SIGNAL(timeout()), this, SLOT(updateTopicList()));
  topic_timer_->start(2000);

  spin_timer_ = new QTimer(this);
  connect(spin_timer_, SIGNAL(timeout()), this, SLOT(spinNode()));
  spin_timer_->start(33);

  updateTopicList();
}

JoystickPanel::~JoystickPanel() {}

void JoystickPanel::initControlTab()
{
  QWidget * tab = new QWidget;
  QVBoxLayout * layout = new QVBoxLayout;

  // --- Enable Switch ---
  enable_check_ = new QCheckBox("Enable Drive (Click Joystick to use WASD)");
  enable_check_->setStyleSheet("QCheckBox { font-weight: bold; font-size: 14px; }");
  enable_check_->setChecked(false);
  layout->addWidget(enable_check_);
  connect(enable_check_, SIGNAL(stateChanged(int)), this, SLOT(onEnableChanged(int)));

  // --- Battery ---
  battery_bar_ = new QProgressBar;
  battery_bar_->setRange(0, 100);
  battery_bar_->setValue(0);
  battery_bar_->setTextVisible(true);
  battery_bar_->setFormat("%p%");
  layout->addWidget(battery_bar_);

  // --- Joystick ---
  joystick_widget_ = new JoystickWidget;
  joystick_widget_->setEnabled(false);
  
  QHBoxLayout * joy_layout = new QHBoxLayout;
  joy_layout->addStretch();
  joy_layout->addWidget(joystick_widget_);
  joy_layout->addStretch();
  layout->addLayout(joy_layout);

  // --- Speak Input ---
  QHBoxLayout * speak_layout = new QHBoxLayout;
  speak_layout->addWidget(new QLabel("Speak:"));
  speak_line_edit_ = new QLineEdit;
  speak_line_edit_->setPlaceholderText("Enter text here...");
  speak_layout->addWidget(speak_line_edit_);
  layout->addLayout(speak_layout);

  connect(speak_line_edit_, SIGNAL(returnPressed()), this, SLOT(sendSpeak()));

  tab->setLayout(layout);
  tab_widget_->addTab(tab, "Control");
}

void JoystickPanel::initSettingsTab()
{
  QWidget * tab = new QWidget;
  QVBoxLayout * layout = new QVBoxLayout;

  QGroupBox * cmd_group = new QGroupBox("Command (Twist) Settings");
  QFormLayout * cmd_form = new QFormLayout;
  cmd_topic_combo_ = new QComboBox; cmd_topic_combo_->setEditable(true);
  cmd_reliability_combo_ = new QComboBox; cmd_reliability_combo_->addItems({"Reliable", "Best Effort"});
  cmd_durability_combo_ = new QComboBox; cmd_durability_combo_->addItems({"Volatile", "Transient Local"});
  cmd_form->addRow("Topic:", cmd_topic_combo_);
  cmd_form->addRow("Reliability:", cmd_reliability_combo_);
  cmd_form->addRow("Durability:", cmd_durability_combo_);
  cmd_group->setLayout(cmd_form);
  layout->addWidget(cmd_group);

  QGroupBox * batt_group = new QGroupBox("Battery Settings");
  QFormLayout * batt_form = new QFormLayout;
  battery_topic_combo_ = new QComboBox; battery_topic_combo_->setEditable(true);
  batt_reliability_combo_ = new QComboBox; batt_reliability_combo_->addItems({"Reliable", "Best Effort"}); batt_reliability_combo_->setCurrentText("Best Effort");
  batt_durability_combo_ = new QComboBox; batt_durability_combo_->addItems({"Volatile", "Transient Local"}); batt_durability_combo_->setCurrentText("Volatile");
  batt_form->addRow("Topic:", battery_topic_combo_);
  batt_form->addRow("Reliability:", batt_reliability_combo_);
  batt_form->addRow("Durability:", batt_durability_combo_);
  batt_group->setLayout(batt_form);
  layout->addWidget(batt_group);

  QGroupBox * speak_group = new QGroupBox("Speak (String) Settings");
  QFormLayout * speak_form = new QFormLayout;
  speak_topic_combo_ = new QComboBox; speak_topic_combo_->setEditable(true);
  speak_reliability_combo_ = new QComboBox; speak_reliability_combo_->addItems({"Reliable", "Best Effort"});
  speak_durability_combo_ = new QComboBox; speak_durability_combo_->addItems({"Volatile", "Transient Local"});
  
  speak_form->addRow("Topic:", speak_topic_combo_);
  speak_form->addRow("Reliability:", speak_reliability_combo_);
  speak_form->addRow("Durability:", speak_durability_combo_);
  speak_group->setLayout(speak_form);
  layout->addWidget(speak_group);

  layout->addStretch();
  tab->setLayout(layout);
  tab_widget_->addTab(tab, "Settings");

  connect(cmd_topic_combo_, SIGNAL(currentTextChanged(QString)), this, SLOT(recreateCmdPublisher()));
  connect(cmd_reliability_combo_, SIGNAL(currentIndexChanged(int)), this, SLOT(recreateCmdPublisher()));
  connect(cmd_durability_combo_, SIGNAL(currentIndexChanged(int)), this, SLOT(recreateCmdPublisher()));

  connect(battery_topic_combo_, SIGNAL(currentTextChanged(QString)), this, SLOT(recreateBatterySubscriber()));
  connect(batt_reliability_combo_, SIGNAL(currentIndexChanged(int)), this, SLOT(recreateBatterySubscriber()));
  connect(batt_durability_combo_, SIGNAL(currentIndexChanged(int)), this, SLOT(recreateBatterySubscriber()));

  connect(speak_topic_combo_, SIGNAL(currentTextChanged(QString)), this, SLOT(recreateSpeakPublisher()));
  connect(speak_reliability_combo_, SIGNAL(currentIndexChanged(int)), this, SLOT(recreateSpeakPublisher()));
  connect(speak_durability_combo_, SIGNAL(currentIndexChanged(int)), this, SLOT(recreateSpeakPublisher()));
}

// ★変更点2: 有効化時にフォーカスをジョイスティックへ移動
void JoystickPanel::onEnableChanged(int state)
{
  bool enabled = (state == Qt::Checked);
  joystick_widget_->setEnabled(enabled);
  
  if (enabled) {
    joystick_widget_->setFocus(); // 即座にフォーカスを渡す
  } else {
    sent_stop_ = false;
  }
}

rclcpp::QoS JoystickPanel::getQoS(const QString & reliability, const QString & durability)
{
  rclcpp::QoS qos(10);
  if (reliability == "Best Effort") qos.best_effort();
  else qos.reliable();

  if (durability == "Transient Local") qos.transient_local();
  else qos.durability_volatile();

  return qos;
}

void JoystickPanel::recreateCmdPublisher()
{
  current_cmd_topic_ = cmd_topic_combo_->currentText();
  if (current_cmd_topic_.isEmpty()) return;
  velocity_publisher_.reset();
  try {
    rclcpp::QoS qos = getQoS(cmd_reliability_combo_->currentText(), cmd_durability_combo_->currentText());
    velocity_publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>(current_cmd_topic_.toStdString(), qos);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "Error creating twist publisher: %s", e.what());
  }
}

void JoystickPanel::recreateBatterySubscriber()
{
  current_battery_topic_ = battery_topic_combo_->currentText();
  if (current_battery_topic_.isEmpty()) return;
  battery_sub_.reset();
  try {
    rclcpp::QoS qos = getQoS(batt_reliability_combo_->currentText(), batt_durability_combo_->currentText());
    auto callback = [this](const sensor_msgs::msg::BatteryState::SharedPtr msg) {
      int percent = static_cast<int>(msg->percentage * 100);
      if (percent < 0) percent = 0; if (percent > 100) percent = 100;
      battery_bar_->setValue(percent);
    };
    battery_sub_ = node_->create_subscription<sensor_msgs::msg::BatteryState>(current_battery_topic_.toStdString(), qos, callback);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "Error creating battery sub: %s", e.what());
  }
}

void JoystickPanel::recreateSpeakPublisher()
{
  current_speak_topic_ = speak_topic_combo_->currentText();
  if (current_speak_topic_.isEmpty()) return;
  speak_publisher_.reset();
  try {
    rclcpp::QoS qos = getQoS(speak_reliability_combo_->currentText(), speak_durability_combo_->currentText());
    speak_publisher_ = node_->create_publisher<std_msgs::msg::String>(current_speak_topic_.toStdString(), qos);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "Error creating speak publisher: %s", e.what());
  }
}

void JoystickPanel::sendSpeak()
{
  QString text = speak_line_edit_->text();
  if (text.isEmpty() || !speak_publisher_) return;
  std_msgs::msg::String msg;
  msg.data = text.toStdString();
  speak_publisher_->publish(msg);
  speak_line_edit_->clear();
}

void JoystickPanel::spinNode() {
  if (rclcpp::ok()) rclcpp::spin_some(node_);
}

void JoystickPanel::sendVel() {
  if (rclcpp::ok() && velocity_publisher_) {
    
    bool drive_enabled = enable_check_->isChecked();
    bool is_active = joystick_widget_->isActive();

    if (drive_enabled) {
      if (is_active) {
        geometry_msgs::msg::Twist msg;
        
        float final_linear = joystick_widget_->getLinear();
        float final_angular = joystick_widget_->getAngular();

        msg.linear.x = final_linear * max_linear_vel_;
        msg.angular.z = final_angular * max_angular_vel_;

        velocity_publisher_->publish(msg);
        sent_stop_ = false;
      }
      else {
        if (!sent_stop_) {
          geometry_msgs::msg::Twist msg;
          msg.linear.x = 0.0;
          msg.angular.z = 0.0;
          velocity_publisher_->publish(msg);
          sent_stop_ = true;
        }
      }
    }
    else {
      if (!sent_stop_) {
        geometry_msgs::msg::Twist msg;
        msg.linear.x = 0.0;
        msg.angular.z = 0.0;
        velocity_publisher_->publish(msg);
        sent_stop_ = true;
      }
    }
  }
}

void JoystickPanel::updateTopicList()
{
  auto topic_names_and_types = node_->get_topic_names_and_types();
  std::set<std::string> twist_topics, battery_topics, string_topics;

  for (const auto & t : topic_names_and_types) {
    for (const auto & type : t.second) {
      if (type == "geometry_msgs/msg/Twist") twist_topics.insert(t.first);
      if (type == "sensor_msgs/msg/BatteryState") battery_topics.insert(t.first);
      if (type == "std_msgs/msg/String") string_topics.insert(t.first);
    }
  }

  auto updateCombo = [](QComboBox* combo, const std::set<std::string>& topics) {
    QString current = combo->currentText();
    bool changed = false;
    if (combo->count() != (int)topics.size()) changed = true;
    else {
      int i = 0;
      for (const auto & topic : topics) {
        if (combo->itemText(i++).toStdString() != topic) { changed = true; break; }
      }
    }
    if (changed) {
      combo->blockSignals(true);
      combo->clear();
      for (const auto & topic : topics) combo->addItem(QString::fromStdString(topic));
      int idx = combo->findText(current);
      if (idx != -1) combo->setCurrentIndex(idx);
      else if (combo->count() > 0 && current.isEmpty()) combo->setCurrentIndex(0);
      combo->blockSignals(false);
      return true;
    }
    return false;
  };

  if (updateCombo(cmd_topic_combo_, twist_topics)) {
    if (cmd_topic_combo_->count() > 0 && current_cmd_topic_.isEmpty()) recreateCmdPublisher();
  }
  if (updateCombo(battery_topic_combo_, battery_topics)) {
    if (battery_topic_combo_->count() > 0 && current_battery_topic_.isEmpty()) recreateBatterySubscriber();
  }
  if (updateCombo(speak_topic_combo_, string_topics)) {
    if (speak_topic_combo_->count() > 0 && current_speak_topic_.isEmpty()) recreateSpeakPublisher();
  }
}

void JoystickPanel::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
  QString str;
  if (config.mapGetString("CmdTopic", &str)) cmd_topic_combo_->setEditText(str);
  if (config.mapGetString("CmdReliability", &str)) cmd_reliability_combo_->setCurrentText(str);
  if (config.mapGetString("CmdDurability", &str)) cmd_durability_combo_->setCurrentText(str);

  if (config.mapGetString("BattTopic", &str)) battery_topic_combo_->setEditText(str);
  if (config.mapGetString("BattReliability", &str)) batt_reliability_combo_->setCurrentText(str);
  if (config.mapGetString("BattDurability", &str)) batt_durability_combo_->setCurrentText(str);

  if (config.mapGetString("SpeakTopic", &str)) speak_topic_combo_->setEditText(str);
  if (config.mapGetString("SpeakReliability", &str)) speak_reliability_combo_->setCurrentText(str);
  if (config.mapGetString("SpeakDurability", &str)) speak_durability_combo_->setCurrentText(str);

  recreateCmdPublisher();
  recreateBatterySubscriber();
  recreateSpeakPublisher();
}

void JoystickPanel::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
  config.mapSetValue("CmdTopic", cmd_topic_combo_->currentText());
  config.mapSetValue("CmdReliability", cmd_reliability_combo_->currentText());
  config.mapSetValue("CmdDurability", cmd_durability_combo_->currentText());

  config.mapSetValue("BattTopic", battery_topic_combo_->currentText());
  config.mapSetValue("BattReliability", batt_reliability_combo_->currentText());
  config.mapSetValue("BattDurability", batt_durability_combo_->currentText());

  config.mapSetValue("SpeakTopic", speak_topic_combo_->currentText());
  config.mapSetValue("SpeakReliability", speak_reliability_combo_->currentText());
  config.mapSetValue("SpeakDurability", speak_durability_combo_->currentText());
}

} // namespace erasers_kachaka_teleop

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(erasers_kachaka_teleop::JoystickPanel, rviz_common::Panel)
