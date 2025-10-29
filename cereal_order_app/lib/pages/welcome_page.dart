import 'package:flutter/material.dart';

class WelcomePage extends StatelessWidget {
  const WelcomePage({super.key});

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: Colors.white,
      body: Row(
          children: [
            // 왼쪽 컨텐츠
            Expanded(
              child: Padding(
                padding: const EdgeInsets.only(left: 120.0),
                child: Column(
                  mainAxisAlignment: MainAxisAlignment.center,
                  crossAxisAlignment: CrossAxisAlignment.start,
                  children: [
                    // 메인 제목
                    const Text(
                      '반가워요\n좋은 오후에요!',
                      style: TextStyle(
                        fontSize: 100,
                        fontWeight: FontWeight.bold,
                        color: Color(0xFF121212),
                        height: 1.333, // 160px line height (160 / 120 = 1.333)
                      ),
                    ),
                    const SizedBox(height: 64),
                    // 시리얼 주문하기 버튼
                    SizedBox(
                      width: 600,
                      height: 100,
                      child: ElevatedButton(
                        onPressed: () {
                          Navigator.pushNamed(context, '/cup-selection');
                        },
                        style: ElevatedButton.styleFrom(
                          backgroundColor: const Color(0xFF0064FF),
                          foregroundColor: Colors.white,
                          padding: const EdgeInsets.symmetric(
                            horizontal: 40,
                            vertical: 16,
                          ),
                          shape: RoundedRectangleBorder(
                            borderRadius: BorderRadius.circular(25), // 더 둥근 모서리
                          ),
                          elevation: 2,
                        ),
                        child: const Text(
                          '시리얼 주문하기',
                          style: TextStyle(
                            fontSize: 32,
                            fontWeight: FontWeight.w500,
                          ),
                        ),
                      ),
                    ),
                  ],
                ),
              ),
            ),
            // 우측 이미지
            Padding(
              padding: const EdgeInsets.only(right: 120.0),
              child: Image.asset(
                'assets/images/welcome-3d.png',
                width: 600,
                height: 600,
                fit: BoxFit.contain,
                errorBuilder: (context, error, stackTrace) {
                  return const SizedBox(width: 600, height: 600);
                },
              ),
            ),
          ],
        ),
    );
  }
}


