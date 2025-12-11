#include "erasers_kachaka_teleop/joystick_panel.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QPainter>
#include <QMouseEvent>
#include <QPolygon>
#include <cmath>

namespace erasers_kachaka_teleop
{

// =========================================================================
// JoystickWidget Implementation
// =========================================================================
JoystickWidget::JoystickWidget(QWidget * parent) : QWidget(parent) {
  // サイズを大きくしたので最小サイズも少し大きく確保
  setMinimumSize(250, 250);
}

void JoystickWidget::resizeEvent(QResizeEvent * /*event*/) {
  center_ = QPoint(width() / 2, height() / 2);
  if (!mouse_pressed_) stick_pos_ = center_;
}

void JoystickWidget::paintEvent(QPaintEvent * /*event*/) {
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);
  
  // 背景クリア
  painter.fillRect(rect(), QColor(240, 240, 240));

  // --- 矢印の描画 ---
  painter.setBrush(QColor(150, 150, 150));
  painter.setPen(Qt::NoPen);

  // 矢印の位置オフセット
  int arrow_dist = joy_radius_ + 20; 
  int arrow_size = 15;

  // 上下左右に矢印を描くために座標変換を利用
  for (int i = 0; i < 4; ++i) {
    painter.save();
    painter.translate(center_);
    painter.rotate(i * 90); // 0, 90, 180, 270度回転

    // Y軸マイナス方向（画面上）へ描画
    QPolygon arrow;
    arrow << QPoint(0, -arrow_dist - arrow_size)         // 先端
          << QPoint(-arrow_size / 2, -arrow_dist)        // 左下
          << QPoint(arrow_size / 2, -arrow_dist);        // 右下
    
    painter.drawPolygon(arrow);
    painter.restore();
  }

  // --- ベース円 ---
  painter.setBrush(QColor(220, 220, 220));
  painter.drawEllipse(center_, joy_radius_, joy_radius_);

  // --- スティック (濃い灰色) ---
  painter.setBrush(QColor(60, 60, 60)); // Dark Gray
  painter.drawEllipse(stick_pos_, stick_radius_, stick_radius_);
}

void JoystickWidget::mousePressEvent(QMouseEvent * event) {
  QPoint diff = event->pos() - center_;
  if (std::sqrt(diff.x()*diff.x() + diff.y()*diff.y()) < joy_radius_) {
    mouse_pressed_ = true;
    updateStickPos(event->pos());
  }
}
void JoystickWidget::mouseMoveEvent(QMouseEvent * event) {
  if (mouse_pressed_) updateStickPos(event->pos());
}
void JoystickWidget::mouseReleaseEvent(QMouseEvent * /*event*/) {
  mouse_pressed_ = false;
  stick_pos_ = center_;
  linear_val_ = 0.0f;
  angular_val_ = 0.0f;
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
  linear_val_ = -((float)(stick_pos_.y() - center_.y()) / joy_radius_);
  angular_val_ = -((float)(stick_pos_.x() - center_.x()) / joy_radius_);
  update();
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

  // Battery
  battery_bar_ = new QProgressBar;
  battery_bar_->setRange(0, 100);
  battery_bar_->setValue(0);
  battery_bar_->setTextVisible(true);
  battery_bar_->setFormat("%p%");
  layout->addWidget(battery_bar_);

  // Joystick
  joystick_widget_ = new JoystickWidget;
  // センタリングするためにレイアウト調整
  QHBoxLayout * joy_layout = new QHBoxLayout;
  joy_layout->addStretch();
  joy_layout->addWidget(joystick_widget_);
  joy_layout->addStretch();
  layout->addLayout(joy_layout);

  // Speak Input
  QHBoxLayout * speak_layout = new QHBoxLayout;
  speak_layout->addWidget(new QLabel("Speak:"));
  speak_line_edit_ = new QLineEdit;
  speak_line_edit_->setPlaceholderText("Enter text here...");
  speak_layout->addWidget(speak_line_edit_);
  layout->addLayout(speak_layout);

  // Enterキーで送信
  connect(speak_line_edit_, SIGNAL(returnPressed()), this, SLOT(sendSpeak()));

  tab->setLayout(layout);
  tab_widget_->addTab(tab, "Control");
}

void JoystickPanel::initSettingsTab()
{
  QWidget * tab = new QWidget;
  QVBoxLayout * layout = new QVBoxLayout;

  // --- Twist Settings ---
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

  // --- Battery Settings ---
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

  // --- Speak Settings ---
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

  // Connect Signals
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
  
  // 送信後にクリアする
  speak_line_edit_->clear();
  RCLCPP_INFO(node_->get_logger(), "Sent speak: %s", msg.data.c_str());
}

void JoystickPanel::spinNode() {
  if (rclcpp::ok()) rclcpp::spin_some(node_);
}

void JoystickPanel::sendVel() {
  if (rclcpp::ok() && velocity_publisher_) {
    geometry_msgs::msg::Twist msg;
    msg.linear.x = joystick_widget_->getLinear() * max_linear_vel_;
    msg.angular.z = joystick_widget_->getAngular() * max_angular_vel_;
    velocity_publisher_->publish(msg);
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
