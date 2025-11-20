/// 앱 설정
class AppConfig {
  /// 디버그 모드 (true면 테스트 버튼 표시)
  static const bool debugMode = true;
  
  /// 연결 상태 표시 여부
  static const bool showConnectionStatus = true;
  
  // ============================================
  // API 설정
  // ============================================
  
  /// API 베이스 URL
  static const String apiBaseUrl = 'http://localhost:8000';
  
  /// API 타임아웃 (초)
  static const int apiTimeoutSeconds = 30;
  
  // ============================================
  // ROS2 설정
  // ============================================
  
  /// ROS2 WebSocket 서버 URL
  static const String ros2ServerUrl = 'ws://localhost:9090';
}

