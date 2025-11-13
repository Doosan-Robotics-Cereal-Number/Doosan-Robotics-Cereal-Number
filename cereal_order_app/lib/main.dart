import 'package:flutter/material.dart';
import 'pages/welcome_page.dart';
import 'pages/cereal_selection_page.dart';
import 'pages/cup_selection_page.dart';
import 'pages/quantity_selection_page.dart';
import 'pages/loading_container_page.dart';
import 'pages/order_complete_page.dart';
import 'pages/voice_order_page.dart';

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
      },
    );
  }
}
