import 'package:flutter/material.dart';
import 'primary_button.dart';

/// 재사용 가능한 선택 화면 레이아웃
class SelectionPageLayout extends StatelessWidget {
  final String title;
  final Widget contentArea;
  final VoidCallback? onBackPressed;
  final String backButtonText;
  final String confirmButtonText;
  final bool isConfirmEnabled;
  final VoidCallback? onConfirmPressed;
  final bool showAppBar;

  const SelectionPageLayout({
    super.key,
    required this.title,
    required this.contentArea,
    this.onBackPressed,
    this.backButtonText = '처음으로',
    this.confirmButtonText = '선택하기',
    this.isConfirmEnabled = false,
    this.onConfirmPressed,
    this.showAppBar = false,
  });

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: const Color(0xFFF5F5F5),
      appBar: showAppBar
          ? AppBar(
              backgroundColor: const Color(0xFFF5F5F5),
              elevation: 0,
              leading: Container(
                margin: const EdgeInsets.only(left: 16, top: 8, bottom: 8),
                child: GestureDetector(
                  onTap: onBackPressed ?? () => Navigator.pop(context),
                  child: Container(
                    width: 200,
                    height: 64,
                    decoration: const BoxDecoration(
                      color: Color(0xFFF8F8F8),
                      border: Border.fromBorderSide(
                        BorderSide(
                          width: 1,
                          color: Color(0xFFDCDCDC),
                        ),
                      ),
                    ),
                    child: Center(
                      child: Text(
                        backButtonText,
                        style: const TextStyle(
                          fontSize: 16,
                          fontWeight: FontWeight.w500,
                          color: Color(0xFF666666),
                        ),
                      ),
                    ),
                  ),
                ),
              ),
            )
          : null,
      body: SafeArea(
        child: Padding(
          padding: const EdgeInsets.symmetric(horizontal: 40.0, vertical: 30.0),
          child: Column(
            children: [
              // 처음으로 버튼 (AppBar가 없을 때만)
              if (!showAppBar)
                Align(
                  alignment: Alignment.topLeft,
                  child: GestureDetector(
                    onTap: onBackPressed ?? () => Navigator.pop(context),
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
                      child: Text(
                        backButtonText,
                        style: const TextStyle(
                          fontSize: 16,
                          fontWeight: FontWeight.w400,
                          color: Color(0xFF666666),
                        ),
                      ),
                    ),
                  ),
                ),
              if (!showAppBar) const SizedBox(height: 60),
              if (showAppBar) const SizedBox(height: 20),
              // 메인 질문
              Text(
                title,
                style: TextStyle(
                  fontSize: showAppBar ? 24 : 32,
                  fontWeight: FontWeight.bold,
                  color: const Color(0xFF333333),
                ),
                textAlign: TextAlign.center,
              ),
              SizedBox(height: showAppBar ? 40 : 80),
              // 컨텐츠 영역
              Expanded(
                child: contentArea,
              ),
              SizedBox(height: showAppBar ? 30 : 60),
              // 확인 버튼
              PrimaryButton(
                text: confirmButtonText,
                isEnabled: isConfirmEnabled,
                onPressed: onConfirmPressed,
                showAppBar: showAppBar,
              ),
              SizedBox(height: showAppBar ? 20 : 30),
            ],
          ),
        ),
      ),
    );
  }
}


