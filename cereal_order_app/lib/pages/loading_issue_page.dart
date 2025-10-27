import 'package:flutter/material.dart';
import '../models/order_data.dart';

class LoadingIssuePage extends StatefulWidget {
  final OrderData? orderData;
  
  const LoadingIssuePage({
    super.key,
    this.orderData,
  });

  @override
  State<LoadingIssuePage> createState() => _LoadingIssuePageState();
}

class _LoadingIssuePageState extends State<LoadingIssuePage> with TickerProviderStateMixin {
  late AnimationController _pulseController;
  
  @override
  void initState() {
    super.initState();
    
    // 경고 아이콘 펄스 애니메이션
    _pulseController = AnimationController(
      duration: const Duration(milliseconds: 1000),
      vsync: this,
    )..repeat(reverse: true);
  }

  @override
  void dispose() {
    _pulseController.dispose();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: const Color(0xFFF5F5F5),
      body: Center(
        child: Padding(
          padding: const EdgeInsets.symmetric(horizontal: 40.0, vertical: 30.0),
          child: Column(
            mainAxisAlignment: MainAxisAlignment.center,
            crossAxisAlignment: CrossAxisAlignment.center,
            children: [
              // 경고 메시지
              const Text(
                '잠시 문제가 발생했어요.',
                style: TextStyle(
                  fontSize: 32,
                  fontWeight: FontWeight.bold,
                  color: Color(0xFFFF6B6B),
                ),
                textAlign: TextAlign.center,
              ),
              const SizedBox(height: 60),
              // 경고 아이콘 (펄스 애니메이션)
              AnimatedBuilder(
                animation: _pulseController,
                builder: (context, child) {
                  return Transform.scale(
                    scale: 1.0 + (_pulseController.value * 0.2),
                    child: Container(
                      width: 120,
                      height: 120,
                      decoration: BoxDecoration(
                        color: Colors.red.shade100,
                        shape: BoxShape.circle,
                      ),
                      child: Icon(
                        Icons.warning_rounded,
                        size: 80,
                        color: Colors.red.shade600,
                      ),
                    ),
                  );
                },
              ),
              const SizedBox(height: 40),
              // 안내 메시지
              const Text(
                '조금만 기다려주세요',
                style: TextStyle(
                  fontSize: 24,
                  color: Colors.black87,
                ),
                textAlign: TextAlign.center,
              ),
              const SizedBox(height: 12),
              const Text(
                '문제를 해결하고 있습니다',
                style: TextStyle(
                  fontSize: 18,
                  color: Colors.black54,
                ),
                textAlign: TextAlign.center,
              ),
            ],
          ),
        ),
      ),
    );
  }
}

