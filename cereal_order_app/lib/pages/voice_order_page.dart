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
  OrderData? orderData;

  @override
  void initState() {
    super.initState();
    _initializeService();
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

    // 서비스 시작
    _statusService.start();

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
}

