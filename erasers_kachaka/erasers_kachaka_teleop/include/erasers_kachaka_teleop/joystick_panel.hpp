#ifndef ERASERS_KACHAKA_TELEOP_JOYSTICK_PANEL_HPP
#define ERASERS_KACHAKA_TELEOP_JOYSTICK_PANEL_HPP

#include <rviz_common/panel.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_msgs/msg/string.hpp>

#include <QWidget>
#include <QComboBox>
#include <QLabel>
#include <QProgressBar>
#include <QTimer>
#include <QTabWidget>
#include <QGroupBox>
#include <QLineEdit>
#include <QCheckBox>
#include <QKeyEvent>

namespace erasers_kachaka_teleop
{

class JoystickWidget : public QWidget
{
  Q_OBJECT
public:
  explicit JoystickWidget(QWidget * parent = nullptr);
  
  float getLinear() const;
  float getAngular() const;
  bool isActive() const;

protected:
  void paintEvent(QPaintEvent * event) override;
  void mouseMoveEvent(QMouseEvent * event) override;
  void mousePressEvent(QMouseEvent * event) override;
  void mouseReleaseEvent(QMouseEvent * event) override;
  void resizeEvent(QResizeEvent * event) override;
  void changeEvent(QEvent * event) override;
  void focusInEvent(QFocusEvent * event) override;
  void focusOutEvent(QFocusEvent * event) override;
  void keyPressEvent(QKeyEvent * event) override;
  void keyReleaseEvent(QKeyEvent * event) override;

private:
  void updateStickPos(const QPoint & pos);
  void updateVelocity(); // キー入力計算用
  void calcStickPosFromVelocity(); // 速度からスティック位置を逆算（キー操作の可視化）

  QPoint center_;
  QPoint stick_pos_;
  bool mouse_pressed_ = false;
  
  float mouse_linear_ = 0.0f;
  float mouse_angular_ = 0.0f;

  float key_linear_ = 0.0f;
  float key_angular_ = 0.0f;
  
  bool w_down_ = false;
  bool a_down_ = false;
  bool s_down_ = false;
  bool d_down_ = false;

  int joy_radius_ = 100;
  int stick_radius_ = 35;
};

class JoystickPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit JoystickPanel(QWidget * parent = nullptr);
  ~JoystickPanel() override;

  void load(const rviz_common::Config & config) override;
  void save(rviz_common::Config config) const override;

protected Q_SLOTS:
  void sendVel();
  void sendSpeak();
  void spinNode();
  void updateTopicList();
  
  void onEnableChanged(int state);

  void recreateCmdPublisher();
  void recreateBatterySubscriber();
  void recreateSpeakPublisher();

private:
  void initControlTab();
  void initSettingsTab();
  rclcpp::QoS getQoS(const QString & reliability, const QString & durability);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr speak_publisher_;

  QTabWidget * tab_widget_;
  
  // Tab 1: Control
  QCheckBox * enable_check_;
  JoystickWidget * joystick_widget_;
  QProgressBar * battery_bar_;
  QLineEdit * speak_line_edit_;

  // Tab 2: Settings
  QComboBox * cmd_topic_combo_;
  QComboBox * battery_topic_combo_;
  QComboBox * speak_topic_combo_;
  
  QComboBox * cmd_reliability_combo_;
  QComboBox * cmd_durability_combo_;
  QComboBox * batt_reliability_combo_;
  QComboBox * batt_durability_combo_;
  QComboBox * speak_reliability_combo_;
  QComboBox * speak_durability_combo_;

  QTimer * output_timer_;
  QTimer * topic_timer_;
  QTimer * spin_timer_;

  QString current_cmd_topic_;
  QString current_battery_topic_;
  QString current_speak_topic_;
  
  bool sent_stop_ = true;

  float max_linear_vel_ = 1.0f;
  float max_angular_vel_ = 1.0f;
};

} // namespace erasers_kachaka_teleop

#endif // ERASERS_KACHAKA_TELEOP_JOYSTICK_PANEL_HPP
