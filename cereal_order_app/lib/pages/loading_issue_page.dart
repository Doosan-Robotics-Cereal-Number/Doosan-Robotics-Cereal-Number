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
      backgroundColor: Colors.white,
      body: SafeArea(
        child: Padding(
          padding: const EdgeInsets.symmetric(horizontal: 40.0, vertical: 30.0),
          child: Column(
            children: [
              const SizedBox(height: 60),
              // 경고 메시지
              const Text(
                '앗! 잠시 문제가 생겼어요!',
                style: TextStyle(
                  fontSize: 32,
                  fontWeight: FontWeight.bold,
                  color: Color(0xFF333333),
                ),
                textAlign: TextAlign.center,
              ),
              const SizedBox(height: 16),
              // 안내 메시지
              const Text(
                '급방 해결해드릴게요.',
                style: TextStyle(
                  fontSize: 24,
                  color: Color(0xFF666666),
                ),
                textAlign: TextAlign.center,
              ),
              const SizedBox(height: 80),
              // 경고 아이콘 (펄스 애니메이션)
              Expanded(
                child: Center(
                  child: AnimatedBuilder(
                    animation: _pulseController,
                    builder: (context, child) {
                      return Transform.scale(
                        scale: 1.0 + (_pulseController.value * 0.1),
                        child: Container(
                          width: 400,
                          height: 400,
                          decoration: BoxDecoration(
                            color: const Color(0xFFFFE5E5),
                            shape: BoxShape.circle,
                          ),
                          child: Icon(
                            Icons.warning_rounded,
                            size: 250,
                            color: const Color(0xFFFF4757),
                          ),
                        ),
                      );
                    },
                  ),
                ),
              ),
              const SizedBox(height: 30),
            ],
          ),
        ),
      ),
    );
  }
}

