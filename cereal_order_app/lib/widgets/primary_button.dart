import 'package:flutter/material.dart';

/// 재사용 가능한 확인 버튼 (메뉴/컵/양 선택하기 버튼)
class PrimaryButton extends StatelessWidget {
  final String text;
  final VoidCallback? onPressed;
  final bool isEnabled;
  final bool showAppBar;

  const PrimaryButton({
    super.key,
    required this.text,
    this.onPressed,
    this.isEnabled = true,
    this.showAppBar = false,
  });

  @override
  Widget build(BuildContext context) {
    return Center(
      child: SizedBox(
        width: 800,
        height: 120,
        child: ElevatedButton(
          onPressed: isEnabled ? onPressed : null,
          style: ElevatedButton.styleFrom(
            backgroundColor: isEnabled
                ? const Color(0xFF0064FF)
                : Colors.grey,
            foregroundColor: Colors.white,
            padding: EdgeInsets.symmetric(
              vertical: showAppBar ? 16 : 20,
            ),
            shape: RoundedRectangleBorder(
              borderRadius: BorderRadius.circular(showAppBar ? 8 : 12),
            ),
            elevation: showAppBar ? 2 : 0,
          ),
          child: Text(
            text,
            style: TextStyle(
              fontSize: showAppBar ? 18 : 22,
              fontWeight: FontWeight.w600,
            ),
          ),
        ),
      ),
    );
  }
}


