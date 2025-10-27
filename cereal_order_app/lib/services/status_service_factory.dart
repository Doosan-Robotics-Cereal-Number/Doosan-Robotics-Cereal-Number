import 'status_service.dart';
import 'manual_status_service.dart';
import 'ros2_status_service.dart';

/// 상태 서비스 타입
enum StatusServiceType {
  manual,   // 수동 버튼 방식
  ros2,     // ROS2 WebSocket 방식
}

/// 상태 서비스 팩토리
class StatusServiceFactory {
  /// 서비스 생성
  static StatusService create(StatusServiceType type, {
    // ROS2 설정 (ros2 타입일 때만 사용)
    String? ros2ServerUrl,
    String? ros2TopicName,
    String? ros2TopicType,
  }) {
    switch (type) {
      case StatusServiceType.manual:
        return ManualStatusService();
        
      case StatusServiceType.ros2:
        return ROS2StatusService(
          serverUrl: ros2ServerUrl ?? 'ws://localhost:9090',
          topicName: ros2TopicName ?? '/robot/status',
          topicType: ros2TopicType ?? 'std_msgs/Int32',
        );
    }
  }
}


