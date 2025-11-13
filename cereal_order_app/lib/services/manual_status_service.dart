import 'dart:async';
import 'status_service.dart';

/// 수동으로 상태를 변경하는 서비스
/// 디버그/테스트용 또는 버튼으로 제어하는 경우 사용
class ManualStatusService implements StatusService {
  final _statusStreamController = StreamController<int>.broadcast();
  final _connectionStreamController = StreamController<bool>.broadcast();
  final _orderDoneStreamController = StreamController<bool>.broadcast();
<<<<<<< HEAD
=======
  final _voiceOrderStartStreamController = StreamController<String>.broadcast();
>>>>>>> feat/16_voice_order_llm_ui
  
  int _currentStatus = 0;
  bool _isActive = false;

  @override
  Stream<int> get statusStream => _statusStreamController.stream;

  @override
  Stream<bool> get connectionStream => _connectionStreamController.stream;

  @override
  Stream<bool> get orderDoneStream => _orderDoneStreamController.stream;

  @override
<<<<<<< HEAD
=======
  Stream<String> get voiceOrderStartStream => _voiceOrderStartStreamController.stream;

  @override
>>>>>>> feat/16_voice_order_llm_ui
  Future<void> start() async {
    print('[Manual] 수동 상태 서비스 시작');
    _isActive = true;
    _connectionStreamController.add(true);
    
    // 초기 상태 전송
    _statusStreamController.add(_currentStatus);
  }

  @override
  void stop() {
    print('[Manual] 수동 상태 서비스 중지');
    _isActive = false;
    _connectionStreamController.add(false);
  }

  @override
  void dispose() {
    _statusStreamController.close();
    _connectionStreamController.close();
    _orderDoneStreamController.close();
<<<<<<< HEAD
=======
    _voiceOrderStartStreamController.close();
>>>>>>> feat/16_voice_order_llm_ui
  }

  @override
  bool get isConnected => _isActive;

  /// 수동으로 상태 변경 (외부에서 호출)
  void setStatus(int status) {
    if (!_isActive) {
      print('[Manual] 서비스가 시작되지 않았습니다');
      return;
    }
    
    print('[Manual] 상태 변경: $_currentStatus → $status');
    _currentStatus = status;
    _statusStreamController.add(_currentStatus);
  }

  /// 현재 상태 조회
  int get currentStatus => _currentStatus;

  /// 수동으로 주문 완료 트리거
  void triggerOrderDone() {
    if (!_isActive) {
      print('[Manual] 서비스가 시작되지 않았습니다');
      return;
    }
    
    print('[Manual] 주문 완료 트리거');
    _orderDoneStreamController.add(true);
  }
  
  /// 주문 정보 발행 (수동 모드에서는 로그만 출력)
  @override
  Future<void> publishOrderInfo({
    required String orderData,
  }) async {
    print('[Manual] 주문 정보 (로그만 출력):');
    print('  - orderData: "$orderData"');
<<<<<<< HEAD
=======
  }

  /// 음성 주문 완료 신호 전송 (수동 모드에서는 로그만 출력)
  @override
  Future<void> publishVoiceOrderDone() async {
    print('[Manual] 음성 주문 완료 (로그만 출력)');
>>>>>>> feat/16_voice_order_llm_ui
  }
}


