import 'dart:async';
import 'dart:convert';
import 'package:web_socket_channel/web_socket_channel.dart';
import 'package:web_socket_channel/status.dart' as status;
import 'status_service.dart';

/// ROS2 WebSocket을 통해 로봇 상태를 받아오는 서비스
class ROS2StatusService implements StatusService {
  // WebSocket 관련
  WebSocketChannel? _channel;
  StreamSubscription? _subscription;
  
  // 스트림 컨트롤러
  final _statusStreamController = StreamController<int>.broadcast();
  final _connectionStreamController = StreamController<bool>.broadcast();
  
  // 설정값
  String _serverUrl;
  String _topicName;
  String _topicType;
  
  // 상태
  bool _isConnected = false;
  Timer? _reconnectTimer;
  int _currentStatus = 0;

  // 생성자
  ROS2StatusService({
    String serverUrl = 'ws://localhost:9090',
    String topicName = '/robot/status',
    String topicType = 'std_msgs/Int32',
  })  : _serverUrl = serverUrl,
        _topicName = topicName,
        _topicType = topicType;

  @override
  Stream<int> get statusStream => _statusStreamController.stream;

  @override
  Stream<bool> get connectionStream => _connectionStreamController.stream;

  @override
  Future<void> start() async {
    print('[ROS2] WebSocket 연결 시작: $_serverUrl');
    await _connect();
  }

  @override
  void stop() {
    print('[ROS2] WebSocket 연결 중지');
    _disconnect();
  }

  @override
  void dispose() {
    _disconnect();
    _statusStreamController.close();
    _connectionStreamController.close();
  }

  @override
  bool get isConnected => _isConnected;

  /// 설정 변경
  void configure({
    String? serverUrl,
    String? topicName,
    String? topicType,
  }) {
    if (serverUrl != null) _serverUrl = serverUrl;
    if (topicName != null) _topicName = topicName;
    if (topicType != null) _topicType = topicType;
  }

  /// WebSocket 연결
  Future<void> _connect() async {
    try {
      _channel = WebSocketChannel.connect(Uri.parse(_serverUrl));
      _isConnected = true;
      _connectionStreamController.add(true);
      print('[ROS2] 연결 성공!');

      // 토픽 구독
      _subscribeToTopic();

      // 메시지 수신 리스닝
      _subscription = _channel!.stream.listen(
        _handleMessage,
        onError: _handleError,
        onDone: _handleDisconnect,
        cancelOnError: false,
      );
    } catch (e) {
      print('[ROS2] 연결 실패: $e');
      _isConnected = false;
      _connectionStreamController.add(false);
      _scheduleReconnect();
    }
  }

  /// ROS2 토픽 구독
  void _subscribeToTopic() {
    if (_channel == null || !_isConnected) return;

    final subscribeMessage = jsonEncode({
      'op': 'subscribe',
      'topic': _topicName,
      'type': _topicType,
    });

    _channel!.sink.add(subscribeMessage);
    print('[ROS2] 토픽 구독: $_topicName (타입: $_topicType)');
  }

  /// 메시지 처리
  void _handleMessage(dynamic message) {
    try {
      final data = jsonDecode(message);
      
      if (data['op'] == 'publish' && data['topic'] == _topicName) {
        // std_msgs/Int32 타입
        int statusFlag = data['msg']['data'] ?? 0;
        
        if (_currentStatus != statusFlag) {
          print('[ROS2] 상태 수신: $_currentStatus → $statusFlag');
          _currentStatus = statusFlag;
          _statusStreamController.add(_currentStatus);
        }
      }
    } catch (e) {
      print('[ROS2] 메시지 파싱 에러: $e');
    }
  }

  /// 에러 처리
  void _handleError(error) {
    print('[ROS2] WebSocket 에러: $error');
    _isConnected = false;
    _connectionStreamController.add(false);
    _scheduleReconnect();
  }

  /// 연결 끊김 처리
  void _handleDisconnect() {
    print('[ROS2] 연결 끊김');
    _isConnected = false;
    _connectionStreamController.add(false);
    _scheduleReconnect();
  }

  /// 재연결 스케줄링
  void _scheduleReconnect() {
    _reconnectTimer?.cancel();
    _reconnectTimer = Timer(const Duration(seconds: 3), () {
      print('[ROS2] 재연결 시도...');
      _connect();
    });
  }

  /// 연결 해제
  void _disconnect() {
    _reconnectTimer?.cancel();
    _subscription?.cancel();
    _channel?.sink.close(status.goingAway);
    _isConnected = false;
    _connectionStreamController.add(false);
  }
}


