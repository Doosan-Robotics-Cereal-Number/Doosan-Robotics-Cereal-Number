import 'dart:async';

/// 로봇 상태 서비스 추상 클래스
/// 다양한 방식으로 로봇 상태를 받아올 수 있도록 인터페이스 정의
abstract class StatusService {
  /// 상태 스트림 (0: 정상, 1: 이슈)
  Stream<int> get statusStream;
  
  /// 연결 상태 스트림 (연결됨/끊김)
  Stream<bool> get connectionStream;
  
  /// 주문 완료 스트림 (true: 완료)
  Stream<bool> get orderDoneStream;
  
  /// 음성 주문 시작 스트림 (String: 시작 메시지)
  Stream<String> get voiceOrderStartStream;
  
  /// 서비스 시작
  Future<void> start();
  
  /// 서비스 중지
  void stop();
  
  /// 리소스 정리
  void dispose();
  
  /// 현재 연결 상태
  bool get isConnected;
  
  /// 주문 정보를 로봇에 전송 (ROS2 모드에서만 동작)
  /// orderData 형식: "start_sequence_a,medium,store" (CSV)
  Future<void> publishOrderInfo({
    required String orderData,
  }) async {
    // 기본 구현: 아무것도 하지 않음 (ManualStatusService에서 사용)
    print('[StatusService] publishOrderInfo 기본 구현 (발행 안 함)');
    print('  - orderData: $orderData');
  }

  /// 음성 주문 완료 신호 전송 (ROS2 모드에서만 동작)
  Future<void> publishVoiceOrderDone() async {
    // 기본 구현: 아무것도 하지 않음
    print('[StatusService] publishVoiceOrderDone 기본 구현 (발행 안 함)');
  }
}


