import 'dart:async';

/// 로봇 상태 서비스 추상 클래스
/// 다양한 방식으로 로봇 상태를 받아올 수 있도록 인터페이스 정의
abstract class StatusService {
  /// 상태 스트림 (0: 정상, 1: 이슈)
  Stream<int> get statusStream;
  
  /// 연결 상태 스트림 (연결됨/끊김)
  Stream<bool> get connectionStream;
  
  /// 서비스 시작
  Future<void> start();
  
  /// 서비스 중지
  void stop();
  
  /// 리소스 정리
  void dispose();
  
  /// 현재 연결 상태
  bool get isConnected;
}


