import 'package:flutter/material.dart';
import 'dart:async';
import '../models/order_data.dart';
import '../services/status_service.dart';
import 'loading_page.dart';

class VoiceOrderLoadingPage extends StatefulWidget {
  final OrderData orderData;
  final StatusService statusService;
  
  const VoiceOrderLoadingPage({
    super.key,
    required this.orderData,
    required this.statusService,
  });

  @override
  State<VoiceOrderLoadingPage> createState() => _VoiceOrderLoadingPageState();
}

class _VoiceOrderLoadingPageState extends State<VoiceOrderLoadingPage> {
  StreamSubscription<bool>? _orderDoneSubscription;

  @override
  void initState() {
    super.initState();
    _initializeService();
  }

  /// 서비스 초기화
  void _initializeService() {
    // 주문 완료 스트림 구독
    _orderDoneSubscription = widget.statusService.orderDoneStream.listen((done) {
      if (mounted && done) {
        print('[VoiceOrderLoadingPage] 주문 완료! OrderCompletePage로 이동');
        Navigator.pushNamed(
          context,
          '/order-complete',
          arguments: widget.orderData,
        );
      }
    });
  }

  @override
  void dispose() {
    _orderDoneSubscription?.cancel();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    // LoadingPage 위젯 재사용
    return LoadingPage(
      orderData: widget.orderData,
      statusService: widget.statusService,
    );
  }
}

