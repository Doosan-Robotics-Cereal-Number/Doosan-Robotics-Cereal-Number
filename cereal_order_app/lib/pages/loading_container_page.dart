import 'package:flutter/material.dart';
import '../models/order_data.dart';
import '../services/status_service.dart';
import '../services/status_service_factory.dart';
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
  StreamSubscription<bool>? _orderDoneSubscription;
  
  // Monitoring App 알림 서비스
  final _monitoringNotifier = MonitoringNotifier();
  
  String _serviceTypeName = '';

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
        if (AppConfig.showConnectionStatus) {
          if (connected) {
            _showSnackBar('✅ $_serviceTypeName 연결됨', Colors.green);
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

    // 주문 완료 스트림 구독
    _orderDoneSubscription = _statusService.orderDoneStream.listen((done) {
      if (mounted && done) {
        print('[LoadingContainerPage] 주문 완료 신호 수신! 5초 후 OrderCompletePage로 이동');
        
        // 5초 딜레이 후 화면 전환
        Future.delayed(const Duration(seconds: 5), () {
          if (mounted) {
            print('[LoadingContainerPage] 5초 경과! OrderCompletePage로 이동');
            Navigator.pushNamed(
              context,
              '/order-complete',
              arguments: orderData,
            );
          }
        });
      }
    });
  }

  /// StatusService getter (LoadingPage에서 접근할 수 있도록)
  StatusService get statusService => _statusService;

  @override
  void dispose() {
    _statusSubscription?.cancel();
    _connectionSubscription?.cancel();
    _orderDoneSubscription?.cancel();
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
                  statusService: _statusService,
                ),
        ),
        
        // 연결 상태 표시 (디버그 모드) - 숨김 처리
        // if (AppConfig.debugMode && AppConfig.showConnectionStatus)
        //   Positioned(
        //     top: 20,
        //     left: 20,
        //     child: _buildConnectionStatus(),
        //   ),
      ],
    );
  }
}
