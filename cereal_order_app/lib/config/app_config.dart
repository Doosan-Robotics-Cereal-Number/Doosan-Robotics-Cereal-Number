import '../services/status_service_factory.dart';

/// 앱 설정
class AppConfig {
  // ============================================
  // 여기서 사용할 상태 서비스 방식 선택!
  // ============================================
  
  /// 현재 사용할 상태 서비스 타입
  static const StatusServiceType currentServiceType = 
      StatusServiceType.ros2;  // manual 또는 ros2로 변경
  
  /// 디버그 모드 (true면 테스트 버튼 표시)
  static const bool debugMode = true;
  
  // ============================================
  // ROS2 설정 (StatusServiceType.ros2 사용시)
  // ============================================
  
  static const String ros2ServerUrl = 'ws://localhost:9090';
  
  // 구독할 토픽 (로봇 → 앱)
  static const String ros2TopicName = '/robot/status';
  static const String ros2TopicType = 'std_msgs/Int32';
  
  // 주문 완료 토픽 (로봇 → 앱)
  static const String orderDoneTopicName = '/dsr01/kiosk/order_done';
  static const String orderDoneTopicType = 'std_msgs/String';
  
  // 발행할 토픽 (앱 → 로봇) - 주문 정보
  static const String orderTopicName = '/dsr01/kiosk/order';
  static const String orderTopicType = 'std_msgs/String';
  
  // 발행할 토픽 (앱 → 로봇) - 음성 주문 시작
  static const String voiceOrderStartTopicName = '/dsr01/kiosk/start_voice_order';
  static const String voiceOrderStartTopicType = 'std_msgs/String';
  
  // ============================================
  // 기타 설정
  // ============================================
  
  /// 연결 상태 표시 여부
  static const bool showConnectionStatus = true;
}


