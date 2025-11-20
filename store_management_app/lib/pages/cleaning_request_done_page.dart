import 'dart:async';

import 'package:flutter/material.dart';
import 'package:lottie/lottie.dart';

class CleaningRequestDonePage extends StatefulWidget {
  const CleaningRequestDonePage({super.key});

  @override
  State<CleaningRequestDonePage> createState() => _CleaningRequestDonePageState();
}

class _CleaningRequestDonePageState extends State<CleaningRequestDonePage> {
  Timer? _autoReturnTimer;
  Timer? _countdownTimer;
  int _secondsRemaining = 3;

  @override
  void initState() {
    super.initState();
    _startAutoReturnTimer();
  }

  void _startAutoReturnTimer() {
    _autoReturnTimer?.cancel();
    _countdownTimer?.cancel();
    _secondsRemaining = 3;

    debugPrint('[CleaningRequestDonePage] 3초 후 Home으로 이동합니다.');
    _countdownTimer = Timer.periodic(const Duration(seconds: 1), (timer) {
      _secondsRemaining--;
      if (_secondsRemaining > 0) {
        debugPrint('[CleaningRequestDonePage] ${_secondsRemaining}초 남음...');
      } else {
        timer.cancel();
      }
    });

    _autoReturnTimer = Timer(const Duration(seconds: 3), () {
      if (!mounted) return;
      debugPrint('[CleaningRequestDonePage] 0초! Home으로 이동합니다.');
      Navigator.of(context).popUntil((route) => route.isFirst);
    });
  }

  @override
  void dispose() {
    _autoReturnTimer?.cancel();
    _countdownTimer?.cancel();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: const Color(0xFF121212),
      body: SafeArea(
        child: Center(
          child: Column(
            mainAxisAlignment: MainAxisAlignment.center,
            children: [
              const SizedBox(height: 40),
              SizedBox(
                width: 200,
                height: 200,
                child: Lottie.asset(
                  'assets/animations/Success.json',
                  repeat: false,
                ),
              ),
              const SizedBox(height: 32),
              const Text(
                '두산이에게 요청을 보냈어요!',
                textAlign: TextAlign.center,
                style: TextStyle(
                  color: Colors.white,
                  fontSize: 26,
                  fontFamily: 'Pretendard',
                  fontWeight: FontWeight.w700,
                  height: 1.33,
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }
}

