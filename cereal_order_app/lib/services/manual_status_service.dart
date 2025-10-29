import 'dart:async';
import 'status_service.dart';

/// 수동으로 상태를 변경하는 서비스
/// 디버그/테스트용 또는 버튼으로 제어하는 경우 사용
class ManualStatusService implements StatusService {
  final _statusStreamController = StreamController<int>.broadcast();
  final _connectionStreamController = StreamController<bool>.broadcast();
  
  int _currentStatus = 0;
  bool _isActive = false;

  @override
  Stream<int> get statusStream => _statusStreamController.stream;

  @override
  Stream<bool> get connectionStream => _connectionStreamController.stream;

  @override
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
  
  /// 주문 정보 발행 (수동 모드에서는 로그만 출력)
  @override
  Future<void> publishOrderInfo({
    required int userCup,
    required String orderDetail,
  }) async {
    print('[Manual] 주문 정보 (로그만 출력):');
    print('  - user_cup: $userCup');
    print('  - order_detail: $orderDetail');
  }
}


