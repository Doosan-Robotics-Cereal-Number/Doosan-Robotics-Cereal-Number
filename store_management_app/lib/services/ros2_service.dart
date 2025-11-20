import 'dart:async';
import 'dart:convert';
import 'package:web_socket_channel/web_socket_channel.dart';
import 'package:web_socket_channel/status.dart' as status;
import '../config/app_config.dart';

/// ROS2 WebSocket을 통해 토픽을 구독하는 서비스
class ROS2Service {
  WebSocketChannel? _channel;
  StreamSubscription? _subscription;
  bool _isConnected = false;

  /// WebSocket 연결
  Future<void> connect() async {
    try {
      print('[ROS2] WebSocket 연결 시도 중: ${AppConfig.ros2ServerUrl}');
      _channel = WebSocketChannel.connect(Uri.parse(AppConfig.ros2ServerUrl));
      _isConnected = true;
      print('[ROS2] 연결 성공!');

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
    }
  }

  /// Cleaning 토픽 구독
  void subscribeToCleaningTopic() {
    if (_channel == null || !_isConnected) {
      print('[ROS2] ❌ Cleaning 토픽 구독 실패: 채널이 없거나 연결되지 않음');
      // 연결이 안 되어 있으면 먼저 연결 시도
      connect().then((_) {
        _subscribeToCleaningTopic();
      });
      return;
    }

    _subscribeToCleaningTopic();
  }

  /// Cleaning 토픽 구독 (내부 메서드)
  void _subscribeToCleaningTopic() {
    if (_channel == null || !_isConnected) {
      print('[ROS2] ❌ Cleaning 토픽 구독 실패: 채널이 없거나 연결되지 않음');
      return;
    }

    try {
      final subscribeMessage = jsonEncode({
        'op': 'subscribe',
        'topic': '/dsr01/cleaning',
        'type': 'std_msgs/String',  // 필요에 따라 타입 변경 가능
      });

      _channel!.sink.add(subscribeMessage);
      print('[ROS2] ✅ Cleaning 토픽 구독: /dsr01/cleaning');
    } catch (e) {
      print('[ROS2] ❌ Cleaning 토픽 구독 중 에러: $e');
    }
  }

  /// 메시지 처리
  void _handleMessage(dynamic message) {
    try {
      print('[ROS2] 수신 메시지: $message');
      final data = jsonDecode(message);
      
      // Cleaning 토픽 메시지 처리
      if (data['op'] == 'publish' && data['topic'] == '/dsr01/cleaning') {
        String msgData = data['msg']['data'] ?? '';
        print('[ROS2] ✅ Cleaning 메시지 수신: "$msgData"');
        // 여기에 cleaning 메시지 처리 로직 추가 가능
      }
    } catch (e) {
      print('[ROS2] ❌ 메시지 파싱 에러: $e');
      print('[ROS2] 원본 메시지: $message');
    }
  }

  /// 에러 처리
  void _handleError(error) {
    print('[ROS2] WebSocket 에러: $error');
    _isConnected = false;
  }

  /// 연결 끊김 처리
  void _handleDisconnect() {
    print('[ROS2] 연결 끊김');
    _isConnected = false;
  }

  /// 연결 해제
  void disconnect() {
    _subscription?.cancel();
    _channel?.sink.close(status.goingAway);
    _isConnected = false;
    print('[ROS2] 연결 해제');
  }

  /// 현재 연결 상태
  bool get isConnected => _isConnected;
}


