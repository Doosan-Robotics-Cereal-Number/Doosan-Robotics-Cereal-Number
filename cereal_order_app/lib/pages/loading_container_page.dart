import 'package:flutter/material.dart';
import '../models/order_data.dart';
import '../services/status_service.dart';
import '../services/status_service_factory.dart';
import '../services/manual_status_service.dart';
import '../services/monitoring_notifier.dart';
import '../config/app_config.dart';
import 'loading_page.dart';
import 'loading_issue_page.dart';
import 'dart:async';

class LoadingContainerPage extends StatefulWidget {
  const LoadingContainerPage({super.key});

  @override
  State<LoadingContainerPage> createState() => _LoadingContainerPageState();
}

class _LoadingContainerPageState extends State<LoadingContainerPage> {
  int issueFlag = 0;
  OrderData? orderData;
  
  // 상태 서비스 (추상 타입으로 선언)
  late StatusService _statusService;
  StreamSubscription<int>? _statusSubscription;
  StreamSubscription<bool>? _connectionSubscription;
  
  // Monitoring App 알림 서비스
  final _monitoringNotifier = MonitoringNotifier();
  
  bool _isConnected = false;
  String _serviceTypeName = '';
  bool _orderPublished = false;  // 주문 정보 발행 여부

  @override
  void initState() {
    super.initState();
    _initializeService();
  }

  @override
  void didChangeDependencies() {
    super.didChangeDependencies();
    // 주문 정보 가져오기
    orderData = ModalRoute.of(context)?.settings.arguments as OrderData?;
    
    // 주문 정보 발행 (한 번만)
    if (!_orderPublished && orderData != null) {
      _publishOrderToRobot();
      _orderPublished = true;
    }
  }

  /// 서비스 초기화
  void _initializeService() {
    // 설정에 따라 서비스 생성
    _statusService = StatusServiceFactory.create(
      AppConfig.currentServiceType,
      ros2ServerUrl: AppConfig.ros2ServerUrl,
      ros2TopicName: AppConfig.ros2TopicName,
      ros2TopicType: AppConfig.ros2TopicType,
    );

    // 서비스 타입 이름 설정
    _serviceTypeName = _getServiceTypeName(AppConfig.currentServiceType);
    print('📱 사용 중인 서비스: $_serviceTypeName');

    // 서비스 시작
    _statusService.start();

    // 연결 상태 구독
    _connectionSubscription = _statusService.connectionStream.listen((connected) {
      if (mounted) {
        setState(() {
          _isConnected = connected;
        });
        
        if (AppConfig.showConnectionStatus) {
          if (connected) {
            _showSnackBar('✅ $_serviceTypeName 연결됨', Colors.green);
            // 연결되면 주문 정보 발행
            if (!_orderPublished && orderData != null) {
              _publishOrderToRobot();
              _orderPublished = true;
            }
          } else {
            _showSnackBar('⚠️ $_serviceTypeName 연결 끊김', Colors.orange);
          }
        }
      }
    });

    // 상태 스트림 구독
    _statusSubscription = _statusService.statusStream.listen((newStatus) {
      if (mounted) {
        setState(() {
          issueFlag = newStatus;
        });
        print('🔄 화면 전환: ${newStatus == 0 ? "정상 ✅" : "이슈 ⚠️"}');
        
        // Monitoring App으로 이슈 알림 전송
        _notifyMonitoringApp(newStatus);
      }
    });
  }

  /// 주문 정보를 로봇에 발행
  Future<void> _publishOrderToRobot() async {
    if (orderData == null) return;

    // 1. user_cup 변환 (Int)
    int userCup = 0;  // 기본값: 매장 컵
    if (orderData!.selectedCup == 'personal') {
      userCup = 1;  // 개인 컵
    }

    // 2. order_detail 생성 (String)
    String cerealType = orderData!.selectedCereal ?? 'cocoball';
    String quantity = orderData!.selectedQuantity ?? '적당히';
    
    // 양 변환 (한글 → 영문)
    String quantityEn = 'normal';  // 기본값
    if (quantity == '많이') {
      quantityEn = 'many';
    } else if (quantity == '적게') {
      quantityEn = 'small';
    }
    
    String orderDetail = '$cerealType,$quantityEn';

    // 3. 로봇에 발행
    print('📤 주문 정보 발행 시작:');
    print('  - user_cup: $userCup (${orderData!.selectedCup})');
    print('  - order_detail: $orderDetail');

    await _statusService.publishOrderInfo(
      userCup: userCup,
      orderDetail: orderDetail,
    );

    print('✅ 주문 정보 발행 완료!');
  }

  @override
  void dispose() {
    _statusSubscription?.cancel();
    _connectionSubscription?.cancel();
    _statusService.stop();
    _statusService.dispose();
    super.dispose();
  }

  /// 서비스 타입 이름
  String _getServiceTypeName(StatusServiceType type) {
    switch (type) {
      case StatusServiceType.manual:
        return '수동 모드';
      case StatusServiceType.ros2:
        return 'ROS2';
    }
  }

  /// 스낵바 표시
  void _showSnackBar(String message, Color color) {
    if (!mounted) return;
    ScaffoldMessenger.of(context).showSnackBar(
      SnackBar(
        content: Text(message),
        backgroundColor: color,
        duration: const Duration(seconds: 2),
      ),
    );
  }
  
  /// Monitoring App으로 이슈 알림 전송
  void _notifyMonitoringApp(int issueFlag) {
    // 주문 정보 생성 (있는 경우)
    String? orderInfo;
    if (orderData != null) {
      orderInfo = '${orderData!.selectedCereal ?? ""} ${orderData!.selectedCup ?? ""} ${orderData!.selectedQuantity ?? ""}개';
    }
    
    // 비동기로 알림 전송 (UI 블로킹 방지)
    _monitoringNotifier.notifyIssue(
      issueFlag: issueFlag,
      orderInfo: orderInfo,
    );
  }

  /// 수동으로 상태 변경 (수동 서비스일 때만 동작)
  void _changeStatusManually(int newStatus) {
    if (_statusService is ManualStatusService) {
      (_statusService as ManualStatusService).setStatus(newStatus);
    } else {
      _showSnackBar('⚠️ 현재 $_serviceTypeName 모드에서는 수동 변경 불가', Colors.orange);
    }
  }

  @override
  Widget build(BuildContext context) {
    orderData = ModalRoute.of(context)?.settings.arguments as OrderData?;

    return Stack(
      children: [
        // 페이지 전환
        AnimatedSwitcher(
          duration: const Duration(milliseconds: 300),
          transitionBuilder: (Widget child, Animation<double> animation) {
            return FadeTransition(
              opacity: animation,
              child: child,
            );
          },
          child: issueFlag == 1
              ? LoadingIssuePage(
                  key: const ValueKey('issue'),
                  orderData: orderData,
                )
              : LoadingPage(
                  key: const ValueKey('normal'),
                  orderData: orderData,
                ),
        ),
        
        // 연결 상태 표시 (디버그 모드) - 숨김 처리
        // if (AppConfig.debugMode && AppConfig.showConnectionStatus)
        //   Positioned(
        //     top: 20,
        //     left: 20,
        //     child: _buildConnectionStatus(),
        //   ),
        
        // 테스트용 버튼 (디버그 모드에서만)
        if (AppConfig.debugMode)
          Positioned(
            bottom: 20,
            right: 20,
            child: _buildDebugButtons(),
          ),
      ],
    );
  }

  /// 연결 상태 위젯
  Widget _buildConnectionStatus() {
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
      decoration: BoxDecoration(
        color: _isConnected 
            ? Colors.green.withOpacity(0.9)
            : Colors.red.withOpacity(0.9),
        borderRadius: BorderRadius.circular(20),
        boxShadow: [
          BoxShadow(
            color: Colors.black.withOpacity(0.2),
            blurRadius: 4,
            offset: const Offset(0, 2),
          ),
        ],
      ),
      child: Row(
        mainAxisSize: MainAxisSize.min,
        children: [
          Icon(
            _isConnected ? Icons.check_circle : Icons.error,
            color: Colors.white,
            size: 18,
          ),
          const SizedBox(width: 8),
          Column(
            crossAxisAlignment: CrossAxisAlignment.start,
            mainAxisSize: MainAxisSize.min,
            children: [
              Text(
                _serviceTypeName,
                style: const TextStyle(
                  color: Colors.white,
                  fontSize: 12,
                  fontWeight: FontWeight.bold,
                ),
              ),
              Text(
                _isConnected ? '연결됨' : '연결 끊김',
                style: const TextStyle(
                  color: Colors.white70,
                  fontSize: 10,
                ),
              ),
            ],
          ),
        ],
      ),
    );
  }

  /// 디버그 버튼들
  Widget _buildDebugButtons() {
    return Column(
      mainAxisSize: MainAxisSize.min,
      children: [
        _buildDebugButton(
          label: '이슈\n발생',
          color: Colors.red,
          icon: Icons.warning,
          onPressed: () => _changeStatusManually(1),
          heroTag: 'issue1',
        ),
        const SizedBox(height: 10),
        _buildDebugButton(
          label: '정상\n복귀',
          color: Colors.green,
          icon: Icons.check,
          onPressed: () => _changeStatusManually(0),
          heroTag: 'issue0',
        ),
      ],
    );
  }

  /// 디버그 버튼 위젯
  Widget _buildDebugButton({
    required String label,
    required Color color,
    required IconData icon,
    required VoidCallback onPressed,
    required String heroTag,
  }) {
    return Container(
      decoration: BoxDecoration(
        shape: BoxShape.circle,
        boxShadow: [
          BoxShadow(
            color: Colors.black.withOpacity(0.3),
            spreadRadius: 2,
            blurRadius: 5,
            offset: const Offset(0, 3),
          ),
        ],
      ),
      child: FloatingActionButton(
        heroTag: heroTag,
        backgroundColor: color,
        onPressed: onPressed,
        child: Column(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            Icon(icon, size: 20, color: Colors.white),
            const SizedBox(height: 2),
            Text(
              label.split('\n')[1],
              style: const TextStyle(
                fontSize: 10,
                fontWeight: FontWeight.bold,
                color: Colors.white,
              ),
            ),
          ],
        ),
      ),
    );
  }
}




