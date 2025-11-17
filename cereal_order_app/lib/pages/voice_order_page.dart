import 'package:flutter/material.dart';
import 'dart:async';
import '../services/status_service.dart';
import '../services/status_service_factory.dart';
import '../config/app_config.dart';
import '../models/order_data.dart';

class VoiceOrderPage extends StatefulWidget {
  const VoiceOrderPage({super.key});

  @override
  State<VoiceOrderPage> createState() => _VoiceOrderPageState();
}

class _VoiceOrderPageState extends State<VoiceOrderPage> {
  late StatusService _statusService;
  StreamSubscription<bool>? _orderDoneSubscription;
  StreamSubscription<Map<String, String>>? _orderInfoSubscription;  // 주문 정보 구독
  StreamSubscription<String>? _orderCancelSubscription;  // 주문 취소 구독
  OrderData? orderData;

  // 주문 정보 표시용 상태 변수
  String? _receivedMenu;
  String? _receivedSize;
  String? _receivedCup;

  @override
  void initState() {
    super.initState();
    _initializeService();
  }

  /// 서비스 초기화
  void _initializeService() async {
    // 설정에 따라 서비스 생성
    _statusService = StatusServiceFactory.create(
      AppConfig.currentServiceType,
      ros2ServerUrl: AppConfig.ros2ServerUrl,
      ros2TopicName: AppConfig.ros2TopicName,
      ros2TopicType: AppConfig.ros2TopicType,
    );

    // 서비스 시작
    await _statusService.start();

    // 음성 주문 시작 신호 발행
    await _statusService.publishVoiceOrderStart();

    // 주문 완료 스트림 구독 (order_done과 동일한 흐름)
    _orderDoneSubscription = _statusService.orderDoneStream.listen((done) {
      if (mounted && done) {
        print('[VoiceOrderPage] 주문 완료! OrderCompletePage로 이동');
        Navigator.pushNamed(
          context,
          '/order-complete',
          arguments: orderData,
        );
      }
    });

    // 주문 정보 스트림 구독
    _orderInfoSubscription = (_statusService as dynamic).orderInfoStream.listen((orderInfo) {
      if (mounted) {
        setState(() {
          _receivedMenu = orderInfo['menu'];
          _receivedSize = orderInfo['size'];
          _receivedCup = orderInfo['cup'];
        });
        print('[VoiceOrderPage] 주문 정보 수신: 메뉴=$_receivedMenu, 양=$_receivedSize, 컵=$_receivedCup');
      }
    });

    // 주문 취소 스트림 구독
    _orderCancelSubscription = (_statusService as dynamic).orderCancelStream.listen((cancelReason) {
      if (mounted) {
        print('[VoiceOrderPage] 주문 취소 수신: $cancelReason');
        // 초기 화면으로 복귀
        Navigator.pop(context);
      }
    });
  }

  @override
  void didChangeDependencies() {
    super.didChangeDependencies();
    // 이전 화면에서 전달받은 주문 데이터 가져오기 (있는 경우)
    orderData = ModalRoute.of(context)?.settings.arguments as OrderData?;
  }

  /// 주문 완료 버튼 클릭
  Future<void> _onOrderComplete() async {
    print('[VoiceOrderPage] 주문 완료하기 버튼 클릭됨');
    await _statusService.publishVoiceOrderDone();
  }

  @override
  void dispose() {
    _orderDoneSubscription?.cancel();
    _orderInfoSubscription?.cancel();  // 주문 정보 구독 취소
    _orderCancelSubscription?.cancel();  // 주문 취소 구독 취소
    _statusService.stop();
    _statusService.dispose();
    super.dispose();
  }

  /// 뒤로가기 버튼 클릭
  void _onBackPressed() {
    print('[VoiceOrderPage] 뒤로가기 버튼 클릭됨');
    Navigator.pop(context);
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: Colors.white,
      body: SafeArea(
        child: Padding(
          padding: const EdgeInsets.symmetric(horizontal: 40.0, vertical: 30.0),
          child: Column(
            children: [
              // 뒤로가기 버튼
              Align(
                alignment: Alignment.topLeft,
                child: GestureDetector(
                  onTap: _onBackPressed,
                  child: Container(
                    padding: const EdgeInsets.symmetric(
                      horizontal: 24,
                      vertical: 12,
                    ),
                    decoration: BoxDecoration(
                      color: Colors.white,
                      border: Border.all(
                        color: const Color(0xFFDCDCDC),
                        width: 1,
                      ),
                      borderRadius: BorderRadius.circular(8),
                    ),
                    child: const Text(
                      '뒤로가기',
                      style: TextStyle(
                        fontSize: 16,
                        fontWeight: FontWeight.w400,
                        color: Color(0xFF666666),
                      ),
                    ),
                  ),
                ),
              ),
              const SizedBox(height: 60),
              // 메인 컨텐츠 영역
              const Spacer(),

              // 주문 정보 표시
              if (_receivedMenu != null && _receivedSize != null && _receivedCup != null)
                Container(
                  padding: const EdgeInsets.all(40),
                  decoration: BoxDecoration(
                    color: Colors.white,
                    borderRadius: BorderRadius.circular(20),
                    border: Border.all(
                      color: const Color(0xFF0064FF),
                      width: 2,
                    ),
                  ),
                  child: Column(
                    children: [
                      const Text(
                        '주문 내역',
                        style: TextStyle(
                          fontSize: 32,
                          fontWeight: FontWeight.bold,
                          color: Color(0xFF121212),
                        ),
                      ),
                      const SizedBox(height: 30),
                      _buildOrderInfoRow('메뉴', _receivedMenu!),
                      const SizedBox(height: 16),
                      _buildOrderInfoRow('양', _receivedSize!),
                      const SizedBox(height: 16),
                      _buildOrderInfoRow('컵', _receivedCup!),
                    ],
                  ),
                ),

              const Spacer(),
              // 주문 완료 버튼
              Center(
                child: SizedBox(
                  width: 800,
                  height: 120,
                  child: ElevatedButton(
                    onPressed: _onOrderComplete,
                    style: ElevatedButton.styleFrom(
                      backgroundColor: const Color(0xFF0064FF),
                      foregroundColor: Colors.white,
                      padding: const EdgeInsets.symmetric(vertical: 20),
                      shape: RoundedRectangleBorder(
                        borderRadius: BorderRadius.circular(12),
                      ),
                      elevation: 0,
                    ),
                    child: const Text(
                      '주문 완료하기',
                      style: TextStyle(
                        fontSize: 22,
                        fontWeight: FontWeight.w600,
                      ),
                    ),
                  ),
                ),
              ),
              const Spacer(),
            ],
          ),
        ),
      ),
    );
  }

  /// 주문 정보 행 위젯
  Widget _buildOrderInfoRow(String label, String value) {
    return Row(
      mainAxisAlignment: MainAxisAlignment.center,
      children: [
        Text(
          '$label: ',
          style: const TextStyle(
            fontSize: 24,
            color: Color(0xFF666666),
          ),
        ),
        Text(
          value,
          style: const TextStyle(
            fontSize: 24,
            fontWeight: FontWeight.w600,
            color: Color(0xFF0064FF),
          ),
        ),
      ],
    );
  }
}

