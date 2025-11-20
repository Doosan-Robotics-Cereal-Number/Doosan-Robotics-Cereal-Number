import 'package:flutter/material.dart';
import 'package:cereal_order_app/pages/welcome_page.dart';
import 'package:cereal_order_app/pages/cereal_selection_page.dart';
import 'package:cereal_order_app/pages/cup_selection_page.dart';
import 'package:cereal_order_app/pages/quantity_selection_page.dart';
import 'package:cereal_order_app/pages/loading_container_page.dart';
import 'package:cereal_order_app/pages/order_complete_page.dart';
import 'package:cereal_order_app/pages/voice_order_page.dart';
import 'package:cereal_order_app/pages/voice_order_loading_page.dart';
import 'package:cereal_order_app/models/order_data.dart';
import 'package:cereal_order_app/services/status_service.dart';

void main() {
  runApp(const CerealOrderApp());
}

class CerealOrderApp extends StatelessWidget {
  const CerealOrderApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: '시리얼 주문 앱',
      theme: ThemeData(
        colorScheme: ColorScheme.fromSeed(seedColor: Colors.blue),
        useMaterial3: true,
        fontFamily: 'Pretendard',
      ),
      home: const WelcomePage(),
      routes: {
        '/cereal-selection': (context) => const CerealSelectionPage(),
        '/cup-selection': (context) => const CupSelectionPage(),
        '/quantity-selection': (context) => const QuantitySelectionPage(),
        '/loading': (context) => const LoadingContainerPage(),
        '/order-complete': (context) => const OrderCompletePage(),
        '/voice-order': (context) => const VoiceOrderPage(),
        '/voice-order-loading': (context) {
          final args = ModalRoute.of(context)?.settings.arguments as Map<String, dynamic>?;
          if (args != null && args['orderData'] != null && args['statusService'] != null) {
            return VoiceOrderLoadingPage(
              orderData: args['orderData'] as OrderData,
              statusService: args['statusService'] as StatusService,
            );
          }
          // 에러 처리: 기본값 반환
          return const Scaffold(
            body: Center(
              child: Text('주문 정보를 찾을 수 없습니다.'),
            ),
          );
        },
      },
    );
  }
}
